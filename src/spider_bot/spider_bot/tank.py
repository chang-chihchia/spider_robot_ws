import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class TankTestController(Node):
    def __init__(self):
        super().__init__('tank_test_controller')
        
        # 1. 發布者：對應 YAML 中的 dc_motor_controller
        # 注意：YAML 定義了 8 個關節，順序為 L1(F,B), L3(F,B), L4(F,B), L6(F,B)
        self.wheel_vel_pub = self.create_publisher(
            Float64MultiArray, 
            '/dc_motor_controller/commands', 
            10
        )

        # 2. 測試參數
        self.test_speed = 15.0  # 增加速度，克服初始摩擦力
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("坦克測試節點啟動：專注於履帶滾輪測試...")

    def control_loop(self):
        wheel_msg = Float64MultiArray()

        # 根據 YAML 的關節順序：
        # Index 0: Leg 1 (右前)
        # Index 1: Leg 3 (右後)
        # Index 2: Leg 4 (左後)
        # Index 3: Leg 6 (左前)
        
        # 為了讓機器人前進，左右兩側的指令可能需要相反
        # 我們先假設右側 (0-3) 為正，左側 (4-7) 為負來測試方向
        v = float(20.0) 
    
    # 假設 L1, L6 正轉, L3, L4 反轉 (對應你的 YAML 順序)
        wheel_msg.data = [-v, v, v, -v]
        
        self.wheel_vel_pub.publish(wheel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TankTestController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("停止測試。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()