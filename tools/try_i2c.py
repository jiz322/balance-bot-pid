import smbus2 as smbus
import struct
import time
import math

# ================= 配置 =================
# 根据你的文档：
I2C_ADDR = 0x23        # 设备地址
REG_EULER = 0x26       # 欧拉角起始寄存器 (Roll, Pitch, Yaw)
REG_ACCEL = 0x04       # 加速度起始寄存器

# 初始化 I2C
try:
    bus = smbus.SMBus(1)
except Exception as e:
    print(f"无法打开 I2C 总线: {e}")
    exit(1)

def read_euler_angles():
    try:
        # 文档说明：
        # 从 0x26 开始读取
        # 包含: Roll(4字节) + Pitch(4字节) + Yaw(4字节) = 共 12 字节
        # 类型: float (浮点数)
        # 顺序: 小端 (Little-endian)
        
        # 1. 连续读取 12 个字节
        data = bus.read_i2c_block_data(I2C_ADDR, REG_EULER, 12)
        
        # 2. 将字节转换为字节串
        data_bytes = bytes(data)
        
        # 3. 解析数据
        # '<' 代表小端模式
        # 'f' 代表 float (4字节)
        # 'fff' 代表连续 3 个 float
        roll, pitch, yaw = struct.unpack('<fff', data_bytes)
        
        return roll, pitch, yaw
    except Exception as e:
        print(f"读取错误: {e}")
        return None, None, None

def read_accel_raw():
    try:
        # 读取加速度原始数据 (Int16)
        # 0x04 开始，XYZ 各 2 字节 = 6 字节
        data = bus.read_i2c_block_data(I2C_ADDR, REG_ACCEL, 6)
        vals = struct.unpack('<hhh', bytes(data))
        return vals
    except:
        return None

# ================= 主程序 =================
print("I2C IMU 启动中 (基于提供的协议文档)...")
print(f"设备地址: 0x{I2C_ADDR:02X}")
print("-" * 60)
print(f"{'Roll (翻滚)':<12} {'Pitch (俯仰)':<12} {'Yaw (偏航)':<12}")
print("-" * 60)

while True:
    # 读取欧拉角
    r, p, y = read_euler_angles()
    
    if r is not None:
        # 文档里的 float 通常直接就是角度值
        print(f"{r:12.4f} {p:12.4f} {y:12.4f}", end='\r')
    
    time.sleep(0.1) # 10Hz 刷新
