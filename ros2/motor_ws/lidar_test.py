import serial
import time

def dump_raw():
    print("🚀 正在连接雷达串口 /dev/ttyS6 (115200)...")
    try:
        ser = serial.Serial('/dev/ttyS6', 115200, timeout=0.5)
    except Exception as e:
        print(f"❌ 串口打开失败: {e}")
        return

    print("📬 正在抓取连续的原始雷达数据流（无任何上层过滤）...")
    print("-" * 80)
    
    # 让串口稳定一下
    time.sleep(0.5)
    if ser.in_waiting > 0:
        ser.read(ser.in_waiting) # 清空旧缓存

    packet_count = 0
    cache = bytearray()

    while packet_count < 30: # 抓取 15 包进行多周期交叉对比
        if ser.in_waiting > 0:
            cache.extend(ser.read(ser.in_waiting))
        
        while len(cache) >= 47:
            # 死死盯住帧头同步特征
            if cache[0] == 0xAA and cache[1] == 0x55:
                packet = cache[:47]
                del cache[:47]
                
                packet_count += 1
                # 打印成最适合肉眼分析的空格隔开的十六进制串
                hex_str = " ".join(f"{b:02x}" for b in packet)
                print(f"[包 {packet_count:02d}] {hex_str}")
            else:
                del cache[0]
        time.sleep(0.002)

    ser.close()
    print("-" * 80)
    print("🏁 抓取完成！")

if __name__ == '__main__':
    dump_raw()
