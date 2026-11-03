# 技能包：FPGA 底层开发的大模型协作与串口极限传输

本技能包提炼了本项目在开发过程中，利用 AI 大模型（DeepSeek）辅助底层 RTL 开发，以及在极端受限环境下（无网线、无 U 盘）进行硬件部署的可复用经验。

## 1. 提示词工作流：利用大模型生成 AXI-Stream 采集状态机
**适用场景**：在 FPGA 开发初期，快速将外部传感器的并行信号（如 DVP 摄像头）转化为标准的 AXI4-Stream 视频流，省去手写复杂状态机的时间。

**已验证的提示词模板**：
> “你现在是一个资深的 FPGA 硬件工程师。请帮我写一个用于 OV5640 摄像头的 DVP 接口采集模块（Verilog）。输入：像素时钟 pclk，帧同步 vsync，行同步 href，8位数据 data_in。输出：符合 AXI4-Stream 协议的视频流（tvalid, tready, tdata[23:0], tlast）。逻辑：在 pclk 上升沿且 href 为高时采样数据，将连续两个 8 位字节拼成 16 位 RGB565，再扩展成 24 位输出。在 vsync 或 href 结束时输出 tlast。请处理好背压信号。”

**踩坑清单（AI代码的修正项）**：
*   **位宽冲突**：AI 常忽略 Xilinx 官方 DMA IP 的最低要求（32位）。直接使用 AI 默认的 24 位会导致 Vivado 综合布线报错 `TDATA_WIDTH` 不匹配，需手动将输出补齐到 32 位。
*   **时序约束**：AI 无法预知你的板卡物理时钟，必须手动编写 XDC 文件绑定引脚，并在 Block Design 中正确连接 `aclk`。

## 2. 极限技能：无网络/无驱动的纯串口分包传输脚本
**适用场景**：当 PYNQ/Zynq 开发板无外网、无 U 盘、且底层 g_ether 驱动缺失时，如何将 PC 端的硬件文件（.hwh / .bit）强制部署至板卡底层。

**实现原理**：
利用 `pyserial` 库，将二进制文件按块（Chunk）切分，通过波特率 115200 的 COM 口发送。板卡端利用 `cat > filename` 命令开启接收管道，实现物理层的直接透传。

**已验证的 Python 核心发送代码（供他人复用）**：
```python
import serial
import time
import os

COM_PORT = 'COM8' # 需根据实际串口修改
BAUD_RATE = 115200
FILE_NAME = 'design_1_wrapper.hwh' 

ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=2)
time.sleep(1)
ser.write(f"cat > {FILE_NAME}\n".encode('utf-8'))
time.sleep(1) # 等待板卡进入接收状态

with open(FILE_NAME, 'rb') as f:
    data = f.read()

chunk_size = 128
for i in range(0, len(data), chunk_size):
    ser.write(data[i:i+chunk_size])
    ser.flush()
    time.sleep(0.01) # 延时极其重要，防止串口溢出丢包

ser.write(b'\x04') # 发送 Ctrl+D 结束输入
ser.close()