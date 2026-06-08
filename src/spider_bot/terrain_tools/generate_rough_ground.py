import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import random

def main():
    rclpy.init()
    node = rclpy.create_node('terrain_spawner_extended')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('正在等待 Gazebo 服務...')

    # --- 參數設定區域 ---
    GRID_SIZE = 15       # 地圖範圍：從 15x15 擴大到 25x25
    SPACING = 0.22       # 方塊間距：稍微加大以防碰撞過於密集
    BLOCK_SIZE = 0.20    # 方塊寬度
    # ------------------

    print(f"開始生成 {GRID_SIZE}x{GRID_SIZE} 的大型崎嶇地形...")
    
    offset = GRID_SIZE / 2.0

    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            name = f'terrain_{i}_{j}'
            x = (i - offset) * SPACING
            y = (j - offset) * SPACING
            z_h = random.uniform(0.02, 0.05) # 崎嶇高度範圍

            sdf = f"""
            <sdf version="1.6">
              <model name="{name}">
                <static>true</static>
                <link name="link">
                  <collision name="collision">
                    <geometry><box><size>{BLOCK_SIZE} {BLOCK_SIZE} {z_h}</size></box></geometry>
                    <surface>
                      <friction>
                        <ode><mu>100</mu><mu2>100</mu2></ode>
                      </friction>
                    </surface>
                  </collision>
                  <visual name="visual">
                    <geometry><box><size>{BLOCK_SIZE} {BLOCK_SIZE} {z_h}</size></box></geometry>
                    <material><ambient>0.4 0.4 0.4 1</ambient></material>
                  </visual>
                </link>
              </model>
            </sdf>
            """

            request = SpawnEntity.Request()
            request.name = name
            request.xml = sdf
            request.initial_pose.position.x = float(x)
            request.initial_pose.position.y = float(y)
            request.initial_pose.position.z = float(z_h / 2.0)

            # 使用非同步呼叫並等待結果，確保 Gazebo 不會因為瞬間請求過多而崩潰
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            
            if i % 5 == 0 and j == 0: # 每完成一行打印一次進度
                print(f"進度: {i}/{GRID_SIZE} ...")

    print(">>> 大型崎嶇地形生成完畢！")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()