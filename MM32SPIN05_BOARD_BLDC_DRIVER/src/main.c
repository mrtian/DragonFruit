#include "BLDC_controller.h"
#include "config.h"
#include "defines.h"
#include "mm32_device.h"
#include "mpu.h"
#include "setup.h"
#include "util.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h> // for abs()

//------------------------------------------------------------------------
// Global variables set here in main.c
//------------------------------------------------------------------------
extern volatile uint32_t buzzerTimer;
volatile uint32_t main_loop_counter;
int16_t batVoltageCalib;  // global variable for calibrated battery voltage
int16_t board_temp_deg_c; // global variable for calibrated temperature in
                          // degrees Celsius

extern ExtY rtY; /* External outputs */
//---------------
extern InputStruct input; // input structure

extern int16_t speedAvg;    // Average measured speed
extern int16_t speedAvgAbs; // Average measured speed in absolute
extern uint8_t
    timeoutFlgSerial; // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem
                      // detected (line disconnected or wrong Rx data)

extern volatile int pwmr;  // global variable for pwm right. -1000 to 1000
extern uint8_t enable;     // global variable for motor enable
extern int16_t batVoltage; // global variable for battery voltage

extern void SystemInit(void);
extern volatile adc_buf_t adc_buffer;

volatile int32_t target_speed = 0;  // 上位机传来的目标转速
volatile int32_t spd_error_sum = 0; // 积分项
// 设定 PID 参数 (放大 256 倍)
// 初始建议：Kp=2.0 (512), Ki=0.1 (26)
int32_t Kp_spd = 512;
int32_t Ki_spd = 26;

//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
typedef struct {
  uint16_t start;
  uint16_t enable;
  uint16_t err_code;
  int16_t cmd;
  int16_t cmd1;
  int16_t speed_meas;
  int16_t speedR_meas;
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t imutemp;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t curDc;
  uint16_t checksum;
} SerialFeedback;

static SerialFeedback Feedback;

static int16_t
    speedRateFixdt; // local fixed-point variable for speed rate limiter
static int32_t
    speedFixdt; // local fixed-point variable for speed low-pass filter

static uint32_t inactivity_timeout_counter;
static uint16_t rate =
    RATE; // Adjustable rate to support multiple drive modes on startup

uint8_t volatile *TXponter;
int8_t volatile TXcouter;

void UART_tx(void) {
  TXcouter = sizeof(Feedback);
  TXponter = (uint8_t *)&Feedback;
  UART1->ISR &= ~UART_ISR_TX_INTF;
  while (!(UART1->CSR & UART_CSR_TXEPT)) {
  };
  UART1->TDR = *TXponter;
  TXcouter--;
  TXponter++;
  UART1->IER |= UART_IER_TXIEN;
}

SerialCommand command_raw;
extern uint16_t timeoutCntSerial_R;
extern uint8_t timeoutFlgSerial_R;
extern SerialCommand commandR;

void UART1_IRQHandler(void) {
  // receiver
  if (UART1->ISR & UART_ISR_RX_INTF) // rx is not empty
  {
    static uint8_t state = 0;
    static uint8_t *buffPointer;
    UART1->ICR |= UART_ICR_RXICLR;
    uint8_t data = UART1->RDR;
    if (state == 0) {
      if (data == (SERIAL_START_FRAME & 0xFF))
        state++;
    } else if (state == 1) {
      if (data == (SERIAL_START_FRAME >> 8)) {
        state++;
        buffPointer = (uint8_t *)&command_raw + 2;
        command_raw.start = SERIAL_START_FRAME;
      } else
        state = 0;
    } else if (state < 8) {
      *buffPointer = data;
      buffPointer++;
      if (state == 7) {
        state = 0;
        uint16_t checksum = (uint16_t)(command_raw.start ^ command_raw.steer ^
                                       command_raw.speed);
        if (command_raw.checksum == checksum) {
          commandR = command_raw;
          timeoutFlgSerial_R = 0; // Clear timeout flag
          timeoutCntSerial_R = 0; // Reset timeout counter
        }
      } else
        state++;
    }
  }
  // transmitter
  else if (UART1->ISR & UART_ISR_TX_INTF) // tx buff null
  {
    UART1->ICR |= UART_ICR_TXICLR;
    TXcouter--;
    if (!(TXcouter > 0))
      UART1->IER &= ~UART_IER_TXIEN; // disable interrupt
    UART1->TDR = *TXponter;
    TXponter++;
  }
}

void LedGame(void) {
  delay_ms(100);
  LED1_ON;
  delay_ms(100);
  LED2_ON;
  delay_ms(100);
  LED3_ON;
  delay_ms(100);
  LED4_ON;
  delay_ms(100);
  LED1_OFF;
  LED2_OFF;
  LED3_OFF;
  LED4_OFF;
}

uint8_t uart_buff[64];

extern int16_t max_cur_phaB, max_cur_phaC, max_cur_DC;
extern int16_t cur_phaB, cur_phaC, cur_DC;
//  extern int16_T Abs5; // 声明外部变量

int main(void) {
  uint32_t buzzerTimer_prev = 0;
  int16_t speed = 0;

  SystemInit();
  RCC->AHBENR &= ~RCC_AHBENR_DMA1EN; // DMA1CLK_DISABLE();
  delay_Init();
  GPIO_Init();
  // for (uint8_t i=0; i<5; i++) LedGame();
  // delay_ms(100);
  // OPAMP_Init();//开启内部运放
  TIM1_Init();
  ADC1_Init();
  BLDC_Init(); // BLDC Controller Init
  UART_Init();

  // Start ADC conversion
  ADC1->CR |= ADC_CR_TRGEN;

  OFF_PORT->BSRR = 1 << OFF_PIN; // Activate Latch

  Input_Lim_Init(); // Input Limitations Init
  Input_Init();     // Input Init

  poweronMelody();

  // 获取 MCU 温度
  int32_t board_temp_adcFixdt =
      adc_buffer.temp << 16; // Fixed-point filter output initialized with
                             // current ADC converted to fixed-point
  int16_t board_temp_adcFilt = adc_buffer.temp;

  LED1_ON;

  // 主控板有开机按钮，按下释放开机
  if (IS_MASTER_BORAD) {
    while (BUTTON_PORT->IDR & (1 << BUTTON_PIN))
      delay_ms(10); // Loop until button is released 等待开机
  } else {
    // 副板的电压采样好像有问题，值不对，总是 1-3V。直接强制使能
    rtY.z_errCode = 0;
    batVoltage = 3601;
    enable = 1;
  }
  LED2_ON;
  // 开启 UART 中断
  UART1->IER |= UART_IER_RX;
  // ================= MPU 唤醒初始化 =================
  delay_ms(200);
  MPU_Init();
  delay_ms(100);

  int16_t accX = 0;
  int16_t accY = 0;
  int16_t accZ = 0;
  int16_t imutemp = 0;
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;

  // =================================================
  while (1) {
    // 1 ms = 16 ticks buzzerTimer
    if (!(buzzerTimer - buzzerTimer_prev > 16 * DELAY_IN_MAIN_LOOP))
      continue;

    // --- 读取 MPU 数据 ---
    if (main_loop_counter % 20 == 0) {
      MPU_Get_Data_All(&accX, &accY, &accZ, &imutemp, &gyroX, &gyroY, &gyroZ);
    }

    if (IS_MASTER_BORAD == 0) {
      enable = 1;
      rtY.z_errCode = 0;
    }

    // 1. 读取原始命令
    readCommand();

    // ####### MOTOR ENABLING: Only if the initial input is very small (for
    // SAFETY) #######
    if (enable == 0 && !rtY.z_errCode && ABS(input.cmd) < 50) {
      beepShort(6); // make 2 beeps indicating the motor enable
      beepShort(4);
      delay_ms(100);
      speedFixdt = 0; // reset filters
      enable = 1;     // enable motors
    }

    // 对输入数据进行滤波
    rateLimiter16(input.cmd, rate, &speedRateFixdt);
    filtLowPass32(speedRateFixdt >> 4, FILTER, &speedFixdt);
    speed = (int16_t)(speedFixdt >> 16); // convert fixed-point to integer
    // pwmr = speed; //原来直接给占空比去掉，换成以下 转速控制的 PID

    // 2. 实现外环转速 PID：  将输入的 cmd 转换为“目标转速”并进行安全限速
    // speed: -MAX_SPEED_LIMIT ~ MAX_SPEED_LIMIT
    // 我们将其映射到目标转速（比如最高 300转）
    target_speed = speed;
    // 如果设置了转速限制
    if (MAX_SPEED_LIMIT > 0) {
      if (target_speed > MAX_SPEED_LIMIT)
        target_speed = MAX_SPEED_LIMIT;
      if (target_speed < -MAX_SPEED_LIMIT)
        target_speed = -MAX_SPEED_LIMIT;
    }

    // 3. 执行本地 PID 控制 每5ms执行一次
    calcAvgSpeed(); //  获取当前实际转速
    int32_t current_speed = -speedAvg;
    int32_t error = target_speed - current_speed;
    // --- 积分计算 ---
    spd_error_sum += error;
    // 抗饱和限幅：防止积分过大导致刹车刹不住或加速过冲
    if (spd_error_sum > 8000)
      spd_error_sum = 8000;
    if (spd_error_sum < -8000)
      spd_error_sum = -8000;

    // 如果命令是 0 且车速很慢，清空积分，防止电机“嗡嗡”响
    if (target_speed == 0 && abs(current_speed) < 5) {
      spd_error_sum = 0;
    }

    // --- PI 运算 (定点数移位) ---
    int32_t pid_out = ((Kp_spd * error) + (Ki_spd * spd_error_sum)) >> 8;

    // --- 最终输出限幅，最大占空比 1000 ---
    if (pid_out > 1000)
      pid_out = 1000;
    if (pid_out < -1000)
      pid_out = -1000;

    // 🟢 关键：直接控制 pwmr！底层 FOC 算法会拿着这个值去发正弦波
    pwmr = pid_out;
    // 获取 MCU 温度
    // 1. 低通滤波平滑噪声 [保持原有逻辑]
    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
    board_temp_adcFilt = (int16_t)(board_temp_adcFixdt >> 16);
    // 2. 将 ADC 原始值转换为毫伏 (mV)
    // MM32 ADC 为 12 位 (4096)
    // 使用 32 位运算防止溢出: (ADC * VREF) / 4096
    int32_t vsense_mv = ((int32_t)board_temp_adcFilt * V_REF_MV) >> 12;
    // 3. 计算温度 (°C * 10)
    // 公式变换: Temp_x10 = 250 + (V25_mv - Vsense_mv) * 10000 / 4801
    // 这里利用了 MM32 的硬件除法器
    board_temp_deg_c = 250 + ((V25_MV - vsense_mv) * 10000) / 48;

    // ####### CALC CALIBRATED BATTERY VOLTAGE #######
    batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;

    // ####### FEEDBACK SERIAL OUT #######
    uint8_t debug_index = 0;
    if (main_loop_counter % 2 == 0) // Send data periodically every 10 ms
    {
      if (!(UART1->CSR & UART_CSR_TXFULL)) // tx is empty
      {
        Feedback.start = SERIAL_START_FRAME;
        Feedback.enable = enable;
        Feedback.err_code = rtY.z_errCode;
        Feedback.cmd = input.cmd;

        Feedback.cmd1 = cur_phaB;
        Feedback.speed_meas = rtY.n_mot;
        Feedback.speedR_meas = cur_phaC;
        // 陀螺仪数据
        Feedback.accX = accX;
        Feedback.accY = accY;
        Feedback.accZ = accZ;
        Feedback.imutemp = imutemp;
        Feedback.gyroX = gyroX;
        Feedback.gyroY = gyroY;
        Feedback.gyroZ = gyroZ;
        Feedback.batVoltage = batVoltageCalib;
        Feedback.boardTemp = board_temp_deg_c;
        Feedback.curDc = cur_DC;
        Feedback.checksum = Feedback.start ^ Feedback.enable ^
                            Feedback.err_code ^ Feedback.cmd ^ Feedback.cmd1 ^
                            Feedback.speed_meas ^ Feedback.speedR_meas ^
                            Feedback.accX ^ Feedback.accY ^ Feedback.accZ ^
                            Feedback.imutemp ^ Feedback.gyroX ^ Feedback.gyroY ^
                            Feedback.gyroZ ^ Feedback.batVoltage ^
                            Feedback.boardTemp ^ Feedback.curDc;
        UART_tx();
      }
    }

    if (IS_MASTER_BORAD) {
      // ####### POWEROFF BY POWER-BUTTON #######
      // 主控板才有的逻辑，副板没有降 12V
      // 的电路，电源来源于主板，主板关机了，副板会自动关机
      if (BUTTON_PORT->IDR & (1 << BUTTON_PIN)) {
        while (BUTTON_PORT->IDR & (1 << BUTTON_PIN)) {
        };
        poweroff();
      }

      // ####### BEEP AND EMERGENCY POWEROFF #######
      if (TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF &&
          speedAvgAbs < 20) // poweroff before mainboard burns OR low bat 3
        poweroff();
      else if (BAT_DEAD_ENABLE && batVoltage < BAT_DEAD && speedAvgAbs < 20)
        poweroff();
      else if (rtY.z_errCode) // 1 beep (low pitch): Motor error, disable motors
      {
        enable = 0;
        beepCount(1, 24, 1);
      } else if (timeoutFlgSerial) // 3 beeps (low pitch): Serial timeout
        beepCount(3, 24, 1);
      else if (TEMP_WARNING_ENABLE &&
               board_temp_deg_c >=
                   TEMP_WARNING) // 5 beeps (low pitch): Mainboard
                                 // temperature warning
        beepCount(5, 24, 1);
      else if (BAT_LVL1_ENABLE &&
               batVoltage < BAT_LVL1) // 1 beep fast (medium pitch): Low bat 1
        beepCount(0, 10, 6);
      else if (BAT_LVL2_ENABLE &&
               batVoltage < BAT_LVL2) // 1 beep slow (medium pitch): Low bat 2
        beepCount(0, 10, 30);
      else if (BEEPS_BACKWARD &&
               (((speed < -50) &&
                 speedAvg <
                     0))) // 1 beep fast (high pitch): Backward spinning motors
        beepCount(0, 5, 1);
      else
        beepCount(0, 0, 0); // do not beep

      // ####### INACTIVITY TIMEOUT #######
      inactivity_timeout_counter++;
      if (abs(speed) > 50)
        inactivity_timeout_counter = 0;

      if (inactivity_timeout_counter >
          (INACTIVITY_TIMEOUT * 60 * 1000) /
              (DELAY_IN_MAIN_LOOP + 1)) // rest of main loop needs maybe 1ms
        poweroff();
    }
    // Update states
    buzzerTimer_prev = buzzerTimer;
    main_loop_counter++;
  }
}
