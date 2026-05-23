import serial
import struct
import threading
import time

SERIAL_PORT = '/dev/tty.usbserial-110'
BAUD_RATE = 115200

current_speed = 0
current_steer = 0
board_enabled = False  # 全局记录主板使能状态
mpu_raw_capture = [0] * 22
def receive_data(ser):
    global board_enabled
    packet_format = '<HHhhhhhhhhhhhhhhHH'
    packet_size = struct.calcsize(packet_format)
    buffer = b''

    while True:
        try:
            buffer += ser.read(ser.in_waiting or 1)
            
            while len(buffer) >= packet_size:
                start_frame = struct.unpack('<H', buffer[:2])[0]
                
                if start_frame == 0xABCD:
                    packet = buffer[:packet_size]
                    buffer = buffer[packet_size:]
                    
                    data = struct.unpack(packet_format, packet)
                    start, enable,err_code, cmd, cmd1, speed_meas, speedR_meas, ax, ay, az,imutemp, gx, gy, gz, batVolt,boardTemp, dcLink, checksum = data
                    
                    # 修复 Bug: 强制所有负数以 16位无符号形式参与异或计算 (与 C 语言行为保持一致)
                    local_checksum = (
                        (start & 0xFFFF) ^
                        (enable & 0xFFFF) ^
                        (err_code & 0xFFFF) ^
                        (cmd & 0xFFFF) ^ 
                        (cmd1 & 0xFFFF) ^ 
                        (speed_meas & 0xFFFF) ^ 
                        (speedR_meas & 0xFFFF) ^ 
                        (ax & 0xFFFF) ^
                        (ay & 0xFFFF) ^
                        (az & 0xFFFF) ^
                        (imutemp & 0xFFFF) ^
                        (gx & 0xFFFF) ^
                        (gy & 0xFFFF) ^
                        (gz & 0xFFFF) ^
                        (batVolt & 0xFFFF) ^ 
                        (boardTemp & 0xFFFF) ^ 
                        (dcLink & 0xFFFF)
                    ) & 0xFFFF
                    
                    if local_checksum == checksum:
                        
                        board_enabled = (enable == 1 and err_code==0)
                        
                        if err_code != 0:
                            status_str = f"[错误码:{err_code}]"
                        elif enable == 1:
                            status_str = "[正常|已使能]"
                        else:
                            status_str = "[正常|未使能]"

                        actual_dc = dcLink if dcLink < 32768 else dcLink - 65536
                        print(f"\r状态:{status_str:12s} | 实速:{speed_meas:5d} | 电压:{batVolt/100:5.2f}V | aX:{ax:4d} | aY:{ay:4d} | aZ:{az:4d} | gX:{gx} | gY:{gx:4d} | gZ:{gz:4d} | itemp:{imutemp/100:5.2f}°C | 母线:{actual_dc:4d}", end="")
                    else:
                        # 听你的建议：一旦错误，立刻打印原始十六进制数据和对比结果
                        print(f"\n[校验和错误] 期望:{checksum:04X} | 本地计算:{local_checksum:04X}")
                        print(f"丢弃的原始数据包(Hex): {packet.hex().upper()}")
                else:
                    buffer = buffer[1:]
        except Exception as e:
            print(f"读取线程出错: {e}")
            break

def send_data(ser):
    global current_speed, current_steer
    pack_format = '<HhhH'
    
    while True:
        try:
            start_frame = 0xABCD
            steer = current_steer
            speed = current_speed
            
            steer_uint = steer & 0xFFFF
            speed_uint = speed & 0xFFFF
            
            checksum = start_frame ^ steer_uint ^ speed_uint
            
            packet = struct.pack(pack_format, start_frame, steer, speed, checksum)
            ser.write(packet)
            
            time.sleep(0.01)
        except Exception:
            break

def main():
    global current_speed, current_steer, board_enabled
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"成功打开串口 {SERIAL_PORT}！")
    except Exception as e:
        print(f"打开串口失败: {e}")
        return

    threading.Thread(target=receive_data, args=(ser,), daemon=True).start()
    threading.Thread(target=send_data, args=(ser,), daemon=True).start()

    print("\n========================================================")
    print("控制终端已启动！")
    print("【零启动保护】程序默认先发送速度 0 触发主板解锁。")
    print("请观察屏幕输出，只有当状态显示为 [正常|已使能] 时，输入速度才有效。")
    print("输入数字设置速度，输入 0 停止，输入 'q' 退出。")
    print("========================================================\n")

    while True:
        try:
            user_input = input()
            
            if user_input.lower() == 'q':
                current_speed = 0
                time.sleep(0.1)
                break
                
            try:
                new_speed = int(user_input)
                # 发送前先判断是否已经使能
                if not board_enabled and abs(new_speed) >= 50:
                    print("\n>>> 警告: 主板未使能或有报错！请先输入 0 解锁主板。 <<<\n")
                else:
                    current_speed = max(-1000, min(1000, new_speed))
                    print(f"\n>>> 设定速度更新为: {current_speed} <<<\n")
            except ValueError:
                print("\n>>> 无效输入，请输入整数或 'q' 退出 <<<\n")
                
        except KeyboardInterrupt:
            current_speed = 0
            time.sleep(0.1)
            break

    print("正在关闭串口并退出...")
    ser.close()

if __name__ == '__main__':
    main()