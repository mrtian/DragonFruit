import serial
import time

def main():
    port = '/dev/ttyS6'
    baud = 115200
    
    print(f"正在初始化串口 {port} @ {baud}...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except Exception as e:
        print(f"❌ 串口打开失败，请检查是否被ROS节点占用或权限不足: {e}")
        return

    # --- 阶段 1：抓取绝对纯净的连续裸流 ---
    print("\n" + "="*30)
    print("【阶段 1】正在抓取 500 字节完全连续的裸数据流...")
    print("="*30)
    raw_stream = bytearray()
    while len(raw_stream) < 500:
        if ser.in_waiting > 0:
            raw_stream.extend(ser.read(ser.in_waiting))
        time.sleep(0.001)
    
    # 打印裸流
    raw_hex = " ".join(f"{b:02x}" for b in raw_stream[:500])
    print(raw_hex)
    print("="*30 + "\n")

    # 清空当前串口缓存，准备阶段 2
    ser.reset_input_buffer()
    time.sleep(0.1)

    # --- 阶段 2：动态断包抓取 100 包 ---
    print("【阶段 2】开始按协议动态断包（目标 100 包）...")
    cache = bytearray()
    packet_count = 0

    while packet_count < 100:
        if ser.in_waiting > 0:
            cache.extend(ser.read(ser.in_waiting))
        else:
            time.sleep(0.001)
            continue

        while len(cache) >= 4:  # 至少能读到第4字节(LSN)
            if cache[0] == 0xAA and cache[1] == 0x55:
                lsn = cache[3]   # 样本数量
                
                # 协议保护：扫地机单包点数通常在 1 到 64 之间
                if lsn == 0 or lsn > 64:
                    del cache[0]
                    continue
                
                expected_len = 10 + lsn * 2  # 动态计算真实包长
                
                if len(cache) < expected_len:
                    break  # 数据还没收全，等下一次读取
                
                # 提到一帧完整的物理包
                packet = cache[:expected_len]
                del cache[:expected_len]
                
                packet_count += 1
                hex_str = " ".join(f"{b:02x}" for b in packet)
                print(f"[包 {packet_count:02d}] 长度={expected_len:02d} 字节 | {hex_str}")
                
                if packet_count >= 100:
                    break
            else:
                del cache[0]  # 单字节步进寻找 aa 55

    ser.close()
    print("\n数据抓取完毕！")

if __name__ == '__main__':
    main()
