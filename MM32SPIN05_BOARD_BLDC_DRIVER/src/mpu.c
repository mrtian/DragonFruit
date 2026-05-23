#include "mpu.h"

// 宽松延时，确保在高速主频下时序依然稳健
void i2c_delay(void) {
    for (volatile int i = 0; i < 150; i++);
}

void i2c_start(void) {
    SDA_H; SCL_H; i2c_delay();
    SDA_L; i2c_delay();
    SCL_L; i2c_delay();
}

void i2c_stop(void) {
    SDA_L; i2c_delay();
    SCL_H; i2c_delay();
    SDA_H; i2c_delay();
}

uint8_t i2c_write_byte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) SDA_H; else SDA_L;
        i2c_delay();
        SCL_H; i2c_delay();
        SCL_L; i2c_delay();
        data <<= 1;
    }
    // 等待 ACK
    SDA_H; i2c_delay();
    SCL_H; i2c_delay();
    uint8_t ack = !SDA_IN;
    SCL_L; i2c_delay();
    return ack;
}

uint8_t i2c_read_byte(uint8_t ack) {
    uint8_t data = 0;
    SDA_H; // 释放 SDA，准备输入
    for (int i = 0; i < 8; i++) {
        data <<= 1;
        SCL_L; i2c_delay();
        SCL_H; i2c_delay();
        if (SDA_IN) data |= 0x01;
    }
    // 发送应答信号
    SCL_L;
    if (ack) SDA_L; else SDA_H;
    i2c_delay();
    SCL_H; i2c_delay();
    SCL_L;
    SDA_H; // 结束后释放 SDA
    i2c_delay();
    return data;
}

void i2c_write_reg(uint8_t reg, uint8_t dat) {
    i2c_start();
    i2c_write_byte(MPU_ADDR << 1);
    i2c_write_byte(reg);
    i2c_write_byte(dat);
    i2c_stop();
}

void MPU_Init(void) {
    // 硬件引脚初始化已在 setup.c 中处理，此处配置寄存器
    i2c_write_reg(0x6B, 0x80); // 软复位
    for(volatile int i=0; i<50000; i++);
    
    i2c_write_reg(0x6B, 0x01); // 唤醒 + 时钟源
    i2c_write_reg(0x6C, 0x00); // 强制开启所有轴
    i2c_write_reg(0x19, 0x07); // 采样率
    i2c_write_reg(0x1A, 0x03); // 低通滤波
    i2c_write_reg(0x1B, 0x08); // 陀螺仪量程 +/-500
    i2c_write_reg(0x1C, 0x00); // 加速度量程 +/-2g
}

void MPU_Get_Data_All(int16_t *ax, int16_t *ay, int16_t *az, int16_t *temp_cent,
                      int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t d[14];

    // 分段读取：第一段，读取加速度 (0x3B-0x40)
    i2c_start();
    i2c_write_byte(MPU_ADDR << 1);
    i2c_write_byte(0x3B);
    i2c_start();
    i2c_write_byte((MPU_ADDR << 1) | 1);
    for(int i=0;i<13;i++){
        d[i] = i2c_read_byte(1); 
    }
    d[13] = i2c_read_byte(0); // AZ_L -> NACK 结束
    i2c_stop();
    // 拼接数据
    *ax = (int16_t)((d[0] << 8) | d[1]);
    *ay = (int16_t)((d[2] << 8) | d[3]);
    *az = (int16_t)((d[4] << 8) | d[5]);
    
    int16_t raw_temp = (int16_t)((d[6] << 8) | d[7]);
    // float temperature = (float)raw_temp / 294.117f + 21.0f;
    *temp_cent = (int16_t)(((int32_t)raw_temp * 100) / 294 + 2100);

    *gx = (int16_t)((d[8] << 8) | d[9]);
    *gy = (int16_t)((d[10] << 8) | d[11]);
    *gz = (int16_t)((d[12] << 8) | d[13]);
}