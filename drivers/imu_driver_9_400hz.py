"""
YIS IMU Driver - 9-Axis Mode (ACC + GYRO + MAG + EULER) @ 400Hz
Protocol: Yesense YIS v2.3
Baud: 460800 | Freq: 400Hz (code 0x0A) | Output: ACC+GYRO+MAG+EULER (0x00F0)
Note: Temperature excluded to stay within 400Hz bandwidth.
"""
import serial
import struct
import time
import threading
import math

# ================= 配置 =================
IMU_PORT = '/dev/ttyAMA0'
BAUD_RATE = 460800

# YIS 协议常量
HEADER = bytes([0x59, 0x53])

# 输出内容位掩码
BIT_ACC   = (1 << 7)   # 加速度
BIT_GYRO  = (1 << 6)   # 角速度
BIT_MAG   = (1 << 5)   # 磁场强度
BIT_EULER = (1 << 4)   # 欧拉角

# 9轴输出: ACC + GYRO + MAG + EULER (不含温度/四元数/时间戳)
OUTPUT_MASK_9AXIS = BIT_ACC | BIT_GYRO | BIT_MAG | BIT_EULER  # 0x00F0

# 数据ID定义
ID_TEMP  = 0x01
ID_ACC   = 0x10
ID_GYRO  = 0x20
ID_MAG_N = 0x30  # 磁场归一化
ID_MAG_R = 0x31  # 磁场原始强度 (mGauss)
ID_EULER = 0x40
ID_QUAT  = 0x41

# 数据缩放因子
SCALE       = 0.000001   # acc, gyro, euler, quat, mag_normalized
SCALE_MAG_R = 0.001      # mag raw (mGauss)
SCALE_TEMP  = 0.01       # temperature


class IMUDriver9Axis:
    def __init__(self, port, baudrate=460800):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        self.frame_count = 0
        self._last_fps_time = 0
        self._last_fps_count = 0
        self.fps = 0.0

        # 数据容器
        self.data = {
            'acc':   [0.0, 0.0, 0.0],   # m/s²
            'gyro':  [0.0, 0.0, 0.0],   # deg/s
            'mag':   [0.0, 0.0, 0.0],   # 磁场 (归一化 或 mGauss, 取决于模块输出)
            'euler': [0.0, 0.0, 0.0],   # deg (pitch, roll, yaw)
        }

    # ─────────── 协议工具函数 ───────────

    @staticmethod
    def _calc_checksum(data):
        """Fletcher 校验和 (CK1, CK2)"""
        ck1, ck2 = 0, 0
        for b in data:
            ck1 = (ck1 + b) & 0xFF
            ck2 = (ck2 + ck1) & 0xFF
        return ck1, ck2

    @staticmethod
    def _build_cmd(data_class, operator, data_bytes):
        """
        构建交互指令帧
        格式: Header(2) + DataClass(1) + OpLen byte1(1) + OpLen byte2(1) + Data(N) + CK1 + CK2
        """
        data_len = len(data_bytes)
        byte1 = ((data_len & 0x1F) << 3) | (operator & 0x07)
        byte2 = (data_len >> 5) & 0xFF
        payload = bytes([data_class, byte1, byte2]) + bytes(data_bytes)
        ck1, ck2 = IMUDriver9Axis._calc_checksum(payload)
        return HEADER + payload + bytes([ck1, ck2])

    def _send_cmd(self, data_class, operator, data_bytes):
        """发送交互指令"""
        cmd = self._build_cmd(data_class, operator, data_bytes)
        if self.ser and self.ser.is_open:
            self.ser.write(cmd)
            time.sleep(0.2)

    # ─────────── 配置函数 ───────────

    def _set_output(self, bitmask):
        """设置输出内容 (写入RAM)"""
        lo = bitmask & 0xFF
        hi = (bitmask >> 8) & 0xFF
        self._send_cmd(0x04, 0x01, [lo, hi])

    def _set_freq(self, code):
        """设置输出频率 (写入RAM)"""
        self._send_cmd(0x03, 0x01, [code])

    def _set_baud(self, code):
        """设置波特率 (写入RAM)"""
        self._send_cmd(0x02, 0x01, [code])

    def _set_mode_ahrs(self):
        """设置算法模式为 AHRS (9轴融合)"""
        self._send_cmd(0x4D, 0x01, [0x02, 0x01])  # 子类型0x02, 值0x01=AHRS

    # ─────────── 打开/关闭 ───────────

    def open(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"[IMU-9Axis] 串口 {self.port} @ {self.baudrate} 打开成功")
            time.sleep(0.5)

            # 配置: AHRS 模式 (9轴融合)
            print("[IMU-9Axis] 设置算法模式: AHRS")
            self._set_mode_ahrs()
            time.sleep(0.3)

            # 配置: 9轴输出 (ACC + GYRO + MAG + EULER, 无温度)
            print("[IMU-9Axis] 设置输出内容: ACC + GYRO + MAG + EULER")
            self._set_output(OUTPUT_MASK_9AXIS)
            time.sleep(0.3)

            # 配置: 400Hz
            print("[IMU-9Axis] 设置输出频率: 400Hz (code 0x0A)")
            self._set_freq(0x0A)
            time.sleep(0.3)

            # 启动读取线程
            self.running = True
            self._last_fps_time = time.time()
            self._last_fps_count = 0
            self.thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.thread.start()
            print("[IMU-9Axis] 初始化完成, 400Hz 9轴模式运行中")
            return True
        except Exception as e:
            print(f"[IMU-9Axis] 打开失败: {e}")
            return False

    def close(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.ser and self.ser.is_open:
            # 恢复100Hz再关闭
            self._set_freq(0x08)
            time.sleep(0.1)
            self.ser.close()
            print("[IMU-9Axis] 串口已关闭")

    # ─────────── 数据读取与解析 ───────────

    def _reader_loop(self):
        """后台持续读取并解析 YIS 协议帧"""
        buf = b''
        while self.running:
            try:
                if self.ser.in_waiting:
                    buf += self.ser.read(self.ser.in_waiting)

                while len(buf) >= 7:  # 最小帧: Header(2)+TID(2)+LEN(1)+CK1(1)+CK2(1)
                    # 寻找帧头 0x59 0x53
                    idx = buf.find(HEADER)
                    if idx < 0:
                        buf = buf[-1:]
                        break
                    if idx > 0:
                        buf = buf[idx:]
                        continue

                    if len(buf) < 5:
                        break

                    # TID: buf[2:4], LEN: buf[4]
                    data_len = buf[4]
                    frame_len = 5 + data_len + 2  # Header(2)+TID(2)+LEN(1)+DATA(N)+CK(2)

                    if len(buf) < frame_len:
                        break

                    frame = buf[:frame_len]
                    buf = buf[frame_len:]

                    # 校验: 范围从 TID 到 DATA 末尾
                    ck_data = frame[2:5 + data_len]
                    ck1, ck2 = self._calc_checksum(ck_data)
                    if ck1 != frame[-2] or ck2 != frame[-1]:
                        continue

                    # 解析有效数据域
                    self._parse_data(frame[5:5 + data_len])
                    self.frame_count += 1

                    # 计算实时FPS
                    now = time.time()
                    dt = now - self._last_fps_time
                    if dt >= 1.0:
                        self.fps = (self.frame_count - self._last_fps_count) / dt
                        self._last_fps_count = self.frame_count
                        self._last_fps_time = now

                time.sleep(0.0002)
            except Exception as e:
                print(f"[IMU-9Axis] 读取错误: {e}")
                time.sleep(0.5)

    def _parse_data(self, payload):
        """解析 TLV 格式数据域"""
        offset = 0
        while offset + 2 <= len(payload):
            data_id = payload[offset]
            data_len = payload[offset + 1]
            offset += 2

            if offset + data_len > len(payload):
                break

            chunk = payload[offset:offset + data_len]
            offset += data_len

            if data_id == ID_ACC and data_len >= 12:
                ax, ay, az = struct.unpack('<iii', chunk[:12])
                with self.lock:
                    self.data['acc'] = [ax * SCALE, ay * SCALE, az * SCALE]

            elif data_id == ID_GYRO and data_len >= 12:
                gx, gy, gz = struct.unpack('<iii', chunk[:12])
                with self.lock:
                    self.data['gyro'] = [gx * SCALE, gy * SCALE, gz * SCALE]

            elif data_id == ID_MAG_N and data_len >= 12:
                mx, my, mz = struct.unpack('<iii', chunk[:12])
                with self.lock:
                    self.data['mag'] = [mx * SCALE, my * SCALE, mz * SCALE]

            elif data_id == ID_MAG_R and data_len >= 12:
                mx, my, mz = struct.unpack('<iii', chunk[:12])
                with self.lock:
                    self.data['mag'] = [mx * SCALE_MAG_R, my * SCALE_MAG_R, mz * SCALE_MAG_R]

            elif data_id == ID_EULER and data_len >= 12:
                pitch, roll, yaw = struct.unpack('<iii', chunk[:12])
                with self.lock:
                    self.data['euler'] = [pitch * SCALE, roll * SCALE, yaw * SCALE]

    # ─────────── 公开接口 ───────────

    def get_data(self):
        """线程安全地获取最新数据副本"""
        with self.lock:
            return {k: list(v) for k, v in self.data.items()}

    def get_acc(self):
        with self.lock:
            return list(self.data['acc'])

    def get_gyro(self):
        with self.lock:
            return list(self.data['gyro'])

    def get_mag(self):
        with self.lock:
            return list(self.data['mag'])

    def get_euler(self):
        with self.lock:
            return list(self.data['euler'])


# ================= 主程序 =================
if __name__ == "__main__":
    imu = IMUDriver9Axis(IMU_PORT, BAUD_RATE)

    if imu.open():
        time.sleep(1)
        print("=" * 120)
        print("  9-Axis 400Hz IMU 实时数据 (ACC + GYRO + MAG + EULER)")
        print("  按 Ctrl+C 停止")
        print("=" * 120)
        header = (f"{'Time':>7} | "
                  f"{'AccX':>8} {'AccY':>8} {'AccZ':>8} | "
                  f"{'GyrX':>8} {'GyrY':>8} {'GyrZ':>8} | "
                  f"{'MagX':>8} {'MagY':>8} {'MagZ':>8} | "
                  f"{'Pitch':>9} {'Roll':>9} {'Yaw':>10} | "
                  f"{'FPS':>6}")
        print(header)
        print("-" * len(header))

        try:
            start_t = time.time()
            while True:
                d = imu.get_data()
                a = d['acc']
                g = d['gyro']
                m = d['mag']
                e = d['euler']
                t = time.time() - start_t

                print(f"{t:6.1f}s | "
                      f"{a[0]:8.4f} {a[1]:8.4f} {a[2]:8.4f} | "
                      f"{g[0]:8.3f} {g[1]:8.3f} {g[2]:8.3f} | "
                      f"{m[0]:8.3f} {m[1]:8.3f} {m[2]:8.3f} | "
                      f"{e[0]:9.3f} {e[1]:9.3f} {e[2]:10.3f} | "
                      f"{imu.fps:6.1f}",
                      end='\r')
                time.sleep(0.05)

        except KeyboardInterrupt:
            print(f"\n\n总帧数: {imu.frame_count}")
        finally:
            imu.close()