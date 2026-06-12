#!/usr/bin/env python3
"""
Epson M-G354 IMU ROS 2 Driver Node — 包内模块
从串口读取 Epson M-G354 数据，发布为 sensor_msgs/msg/Imu 消息给 RViz2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import time
import struct
import math


class G354IMUNode(Node):
    """Epson M-G354 IMU 驱动节点"""

    def __init__(self):
        super().__init__('g354_imu_node')

        # ---------- 声明参数 ----------
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 460800)
        self.declare_parameter('frame_id', 'imu_link')

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        self.frame_id_ = self.get_parameter('frame_id').value

        # ---------- 创建发布器 ----------
        # rviz_imu_plugin 默认订阅 /imu/data
        self.imu_pub_ = self.create_publisher(Imu, '/imu/data', 10)

        # ---------- 打开串口 ----------
        try:
            self.ser_ = serial.Serial(
                port, baud, timeout=1, rtscts=False, dsrdtr=False
            )
            self.get_logger().info(f'✓ 串口 {port} 已打开 ({baud} baud)')
        except Exception as e:
            self.get_logger().fatal(f'✗ 无法打开串口 {port}: {e}')
            raise

        # ---------- 配置 IMU ----------
        self._configure_imu()

        # ---------- 缓冲区 ----------
        self.buffer_ = bytearray()

        # ---------- 互补滤波器状态 ----------
        self.roll_ = 0.0
        self.pitch_ = 0.0
        self.yaw_ = 0.0
        self.first_frame_ = True           # 首帧用加速度计初始化
        self.prev_stamp_ = None            # 上一帧时间戳，用于 dt

        # ---------- 定时器周期发布 ----------
        # M-G354 配置为 125 Sps，用 8ms 定时器读取
        self.timer_ = self.create_timer(0.008, self.timer_callback)

    # ------------------------------------------------------------------
    def _configure_imu(self):
        """发送 M-G354 标准 UART 自动模式配置序列"""
        instructions = [
            "FE 01 0D",  # 切换到 Window 1
            "85 04 0D",  # 输出速率 125 Sps
            "88 01 0D",  # 开启 UART 自动输出模式
            "8C 06 0D",  # 包含 GPIO, COUNT
            "8D F0 0D",  # 包含 FLAG, TEMP, GYRO, ACCL
            "8F 70 0D",  # 32-bit 分辨率输出
            "FE 00 0D",  # 切换回 Window 0
            "83 01 0D",  # 进入采样模式
        ]
        for cmd in instructions:
            self.ser_.write(bytes.fromhex(cmd))
            time.sleep(0.06)
        self.get_logger().info('✓ IMU 配置完成，已进入采样模式')

    # ------------------------------------------------------------------
    def _complementary_update(self, gx_rad, gy_rad, gz_rad, ax_g, ay_g, az_g, dt):
        """
        互补滤波更新姿态

        原理:
          - 陀螺仪积分 → 三轴高频姿态变化（短时准确）
          - 加速度计 → roll/pitch 绝对参考（长时稳定，不受漂移影响）
          - alpha = 0.98 让陀螺仪主导动态响应，加速度计缓慢修正漂移
          - yaw 无磁力计参考，纯靠陀螺仪积分（慢慢漂是正常的）

        返回 (qw, qx, qy, qz)
        """
        # --- 从加速度计计算 roll/pitch 参考值 ---
        norm = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)
        if norm > 1e-6:
            ax_n, ay_n, az_n = ax_g / norm, ay_g / norm, az_g / norm
            acc_roll = math.atan2(ay_n, az_n)
            acc_pitch = -math.atan2(ax_n, math.sqrt(ay_n * ay_n + az_n * az_n))
        else:
            acc_roll, acc_pitch = 0.0, 0.0

        if self.first_frame_:
            # 首帧用加速度计初始化，避免从 (0,0,0) 跳变
            self.roll_ = acc_roll
            self.pitch_ = acc_pitch
            self.yaw_ = 0.0
            self.first_frame_ = False
        else:
            # --- 陀螺仪积分（高频更新） ---
            self.roll_  += gx_rad * dt
            self.pitch_ += gy_rad * dt
            self.yaw_   += gz_rad * dt

            # --- 互补滤波：加速度计修正 roll/pitch 漂移 ---
            alpha = 0.98   # 陀螺仪权重（越接近 1，对加速度计修正越慢）
            self.roll_  = alpha * self.roll_  + (1.0 - alpha) * acc_roll
            self.pitch_ = alpha * self.pitch_ + (1.0 - alpha) * acc_pitch
            # yaw 无修正源，纯积分（会缓慢漂移）

        # --- 欧拉角 → 四元数 (ZYX 顺序) ---
        cy = math.cos(self.yaw_ * 0.5)
        sy = math.sin(self.yaw_ * 0.5)
        cp = math.cos(self.pitch_ * 0.5)
        sp = math.sin(self.pitch_ * 0.5)
        cr = math.cos(self.roll_ * 0.5)
        sr = math.sin(self.roll_ * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = cy * sp * cr + sy * cp * sr
        qz = sy * cp * cr - cy * sp * sr

        return qw, qx, qy, qz

    # ------------------------------------------------------------------
    def timer_callback(self):
        """定时器回调：读取串口数据、解析、发布 IMU 消息"""
        if self.ser_.in_waiting > 0:
            rx = self.ser_.read(self.ser_.in_waiting)
            self.buffer_.extend(rx)

        while len(self.buffer_) >= 36:
            if self.buffer_[0] == 0x80 and self.buffer_[35] == 0x0D:
                pkt = bytes(self.buffer_[:36])
                del self.buffer_[:36]

                temp_raw = struct.unpack('>h', pkt[3:5])[0]
                gx_raw   = struct.unpack('>h', pkt[7:9])[0]
                gy_raw   = struct.unpack('>h', pkt[11:13])[0]
                gz_raw   = struct.unpack('>h', pkt[15:17])[0]
                ax_raw   = struct.unpack('>h', pkt[19:21])[0]
                ay_raw   = struct.unpack('>h', pkt[23:25])[0]
                az_raw   = struct.unpack('>h', pkt[27:29])[0]

                # 标度因数换算
                gyro_x_dps = gx_raw * 0.016    # °/s
                gyro_y_dps = gy_raw * 0.016
                gyro_z_dps = gz_raw * 0.016
                acc_x_g    = ax_raw * 0.0002   # G
                acc_y_g    = ay_raw * 0.0002
                acc_z_g    = az_raw * 0.0002

                # 转 ROS 标准单位
                gx_rad = gyro_x_dps * 0.017453292519943   # °/s → rad/s
                gy_rad = gyro_y_dps * 0.017453292519943
                gz_rad = gyro_z_dps * 0.017453292519943

                ax_ms2 = acc_x_g * 9.80665     # G → m/s²
                ay_ms2 = acc_y_g * 9.80665
                az_ms2 = acc_z_g * 9.80665

                # --- 互补滤波估计姿态 ---
                now = self.get_clock().now()
                if self.prev_stamp_ is None:
                    dt = 0.008                   # 首帧取默认间隔
                else:
                    dt = (now - self.prev_stamp_).nanoseconds * 1e-9
                    dt = max(0.001, min(dt, 0.05))  # 限幅 [1ms, 50ms]
                self.prev_stamp_ = now

                qw, qx, qy, qz = self._complementary_update(
                    gx_rad, gy_rad, gz_rad,
                    acc_x_g, acc_y_g, acc_z_g,
                    dt
                )

                # 构建 IMU 消息
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id_

                msg.orientation.w = qw
                msg.orientation.x = qx
                msg.orientation.y = qy
                msg.orientation.z = qz
                # covariance[0] ≠ -1 则 RViz 显示姿态轴
                msg.orientation_covariance[0] = 0.01
                msg.orientation_covariance[4] = 0.01
                msg.orientation_covariance[8] = 0.5

                msg.angular_velocity.x = gx_rad
                msg.angular_velocity.y = gy_rad
                msg.angular_velocity.z = gz_rad
                msg.angular_velocity_covariance[0] = 0.0001
                msg.angular_velocity_covariance[4] = 0.0001
                msg.angular_velocity_covariance[8] = 0.0001

                msg.linear_acceleration.x = ax_ms2
                msg.linear_acceleration.y = ay_ms2
                msg.linear_acceleration.z = az_ms2
                msg.linear_acceleration_covariance[0] = 0.0001
                msg.linear_acceleration_covariance[4] = 0.0001
                msg.linear_acceleration_covariance[8] = 0.0001

                self.imu_pub_.publish(msg)

            else:
                self.buffer_.pop(0)

    # ------------------------------------------------------------------
    def destroy_node(self):
        """退出时让 IMU 回到配置模式，关闭串口"""
        try:
            self.ser_.write(bytes.fromhex("83 02 0D"))
            self.ser_.close()
            self.get_logger().info('✓ 串口已安全关闭')
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = G354IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断，正在退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
