import serial
import time
import os

COM_PORT = 'COM8'  # 如果你的串口号变了，记得改这里
BAUD_RATE = 115200
FILE_NAME = 'design_1_wrapper.hwh'

# 1. 连接板子
print("正在打开串口...")
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=2)
time.sleep(1)

if not os.path.exists(FILE_NAME):
    print(f"找不到文件 {FILE_NAME}！")
    exit()

# 2. 先发送接收指令（相当于在黑框里敲 cat > file）
print(f"告诉板子准备接收文件...")
ser.write(f"cat > {FILE_NAME}\n".encode('utf-8'))
time.sleep(1) # 等板子进入接收状态

# 3. 读取并发送文件内容
with open(FILE_NAME, 'rb') as f:
    data = f.read()

print(f"开始传输 {FILE_NAME}，大小: {len(data)} 字节...")

chunk_size = 256 # 每次发256字节
for i in range(0, len(data), chunk_size):
    ser.write(data[i:i+chunk_size])
    ser.flush()
    time.sleep(0.01) # 延时很重要，否则串口会溢出丢数据

# 4. 发送结束符 (Ctrl + D，ASCII码是 4)
ser.write(b'\x04')
print("🎉 传输完成！")

ser.close()