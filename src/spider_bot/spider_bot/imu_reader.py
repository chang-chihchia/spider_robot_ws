import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import socket
import struct
import math

class WitImuUdpNode(Node):
    def __init__(self):
        super().__init__('wit_imu_node')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        
        # 建立 UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 9001))
        self.sock.setblocking(False)
        
        self.buffer = bytearray()
        self.timer = self.create_timer(0.01, self.receive_loop) # 100Hz
        self.get_logger().info('==== 維特 IMU 完整解析節點啟動 (BLE 5.0 模式) ====')

    def euler_to_quaternion(self, roll, pitch, yaw):
        """將歐拉角 (弧度) 轉換為四元數"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0.0] * 4
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        q[3] = cr * cp * cy + sr * sp * sy  # w
        
        # 關鍵：歸一化 (Normalization)
        # 如果沒做這步，RViz 經常會因為數值微小誤差而「拒絕」旋轉
        mag = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
        if mag == 0:
            return [0.0, 0.0, 0.0, 1.0]
        return [q[0]/mag, q[1]/mag, q[2]/mag, q[3]/mag]

    def receive_loop(self):
        try:
            data, addr = self.sock.recvfrom(2048)
            self.buffer.extend(data)
            
            # 尋找 0x61 綜合包 (20 bytes)
            while len(self.buffer) >= 20:
                if self.buffer[0] == 0x55 and self.buffer[1] == 0x61:
                    pack = self.buffer[:20]
                    self.parse_wit_61(pack)
                    del self.buffer[:20]
                else:
                    del self.buffer[0]
        except BlockingIOError:
            pass

    def parse_wit_61(self, pack):
        try:
            # 解析原始數據 (Little-endian short)
            # pack[2:8] 加速度, pack[8:14] 角速度, pack[14:20] 角度
            ax, ay, az = struct.unpack('<hhh', pack[2:8])
            gx, gy, gz = struct.unpack('<hhh', pack[8:14])
            r, p, y = struct.unpack('<hhh', pack[14:20])

            # 轉換為標準單位
            # 角度轉弧度
            roll_rad = (r / 32768.0 * 180.0) * (math.pi / 180.0)
            pitch_rad = (p / 32768.0 * 180.0) * (math.pi / 180.0)
            yaw_rad = (y / 32768.0 * 180.0) * (math.pi / 180.0)

            # 封裝 ROS 2 Imu 消息
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # 1. 填充四元數 (姿態)
            q = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q

            # 2. 填充角速度 (rad/s)
            msg.angular_velocity.x = (gx / 32768.0 * 2000.0) * (math.pi / 180.0)
            msg.angular_velocity.y = (gy / 32768.0 * 2000.0) * (math.pi / 180.0)
            msg.angular_velocity.z = (gz / 32768.0 * 2000.0) * (math.pi / 180.0)

            # 3. 填充線加速度 (m/s^2)
            msg.linear_acceleration.x = (ax / 32768.0 * 16.0) * 9.80665
            msg.linear_acceleration.y = (ay / 32768.0 * 16.0) * 9.80665
            msg.linear_acceleration.z = (az / 32768.0 * 16.0) * 9.80665

            self.publisher_.publish(msg)
            
            # 終端機顯示 (Debug 用)
            self.get_logger().info(
                f'角度(度) -> R:{roll_rad*180/math.pi:.1f} P:{pitch_rad*180/math.pi:.1f} Y:{yaw_rad*180/math.pi:.1f}',
                once=False
            )
            
        except Exception as e:
            self.get_logger().error(f"解析錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WitImuUdpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()