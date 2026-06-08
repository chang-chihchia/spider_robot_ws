import rclpy
import math
import time
import numpy as np
import csv
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from spider_bot.SpiderBotDriver import *
from spider_bot.SpiderBotLib import *

def compute_fk_standalone(thetas, h):
    # 請確保這裡的 L2, L3 與你的機器人物理長度一致
    t1, t2, t3 = thetas
    L2, L3 = 0.10, 0.15 
    
    # 這是標準 D 型軌跡的運動學映射
    y = (L2 * np.cos(t2) + L3 * np.cos(t2 + t3))
    z = L2 * np.sin(t2) + L3 * np.sin(t2 + t3)
    return [float(y), float(z)]

def main(args=None):
    h = SpiderBotLib()
    a = SpiderBotDriver()
    if not rclpy.ok(): rclpy.init()
    ros_node = rclpy.create_node('spider_trajectory_clean_log')

    # CSV 初始化：記錄 Y 和 Z (單位為 mm)
    csv_file = open('/home/chia/spider_ws/end_effector_trajectory_clean.csv', 'w', newline='')
    writer = csv.writer(csv_file)
    writer.writerow(['time', 'L1_y', 'L1_z', 'L4_y', 'L4_z'])

    leg_offsets = [[0.1, 0.1, 0], [-0.1, 0.1, 0], [-0.15, 0, 0], [-0.1, -0.1, 0], [0.1, -0.1, 0], [0.15, 0, 0]]
    leg_angles = [45.0, 135.0, 180.0, 225.0, 315.0, 0.0]
    TARGET_Z, EXTEND_R, stride_period = -0.08, 0.23, 2.0
    step_length, step_height, PUSH_FORCE_X = 0.1, 0.07, 25.0

    def js_callback(msg):
        for i, name in enumerate(msg.name):
            try:
                joint_id = int(''.join(filter(str.isdigit, name)))
                if joint_id in a.joint_cur_pos:
                    a.joint_cur_pos[joint_id]['pos'] = np.degrees(msg.position[i])
                    a.joint_cur_pos[joint_id]['vel'] = np.degrees(msg.velocity[i])
            except: pass

    ros_node.create_subscription(JointState, '/joint_states', js_callback, 10)
    pub = ros_node.create_publisher(Float64MultiArray, '/spider_leg_controller/commands', 10)

    try:
        t_start = time.time()
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.001)
            elapsed = time.time() - t_start
            phase = (elapsed % stride_period) / stride_period

            all_leg_commands = []
            log_data = [] # 用來存這一個時間點的 [L1_y, L1_z, L4_y, L4_z]

            for i in range(1, 7):
                gamma = np.radians(leg_angles[i - 1])
                group_offset = 0.5 if i in [2, 4, 6] else 0.0
                leg_phase = (phase + group_offset) % 1.0
                is_stance = leg_phase >= 0.5
                
                # 步態計算
                if not is_stance:
                    p = leg_phase / 0.5
                    dx = -step_length / 2 + step_length * p
                    dz = step_height * math.sin(math.pi * p)
                    K_mat, D_mat = np.diag([70.0, 70.0, 160.0]), np.diag([7.0, 7.0, 16.0])
                else:
                    p = (leg_phase - 0.5) / 0.5
                    dx = step_length / 2 - step_length * p
                    dz = 0.0
                    K_mat, D_mat = np.diag([140.0, 140.0, 260.0]), np.diag([12.0, 12.0, 24.0])

                target_xyz = np.array([EXTEND_R * math.cos(gamma) + dx, EXTEND_R * math.sin(gamma), TARGET_Z + dz]) + np.array(leg_offsets[i - 1])
                thetas = [np.radians(a.joint_cur_pos[(i - 1) * 3 + 1 + j]['pos']) for j in range(3)]
                omegas = [np.radians(a.joint_cur_pos[(i - 1) * 3 + 1 + j]['vel']) for j in range(3)]
                
                tau = h.compute_impedance_control(target_xyz, np.array(thetas), np.array(omegas), K_mat, D_mat, leg_index=i)
                if is_stance:
                    f_vector_local = np.array([-PUSH_FORCE_X * math.cos(-gamma), -PUSH_FORCE_X * math.sin(-gamma), -8.0])
                    tau += h.force_to_torque(f_vector_local, *thetas)
                
                all_leg_commands.extend(np.clip(tau, -7.0, 7.0))
                
                # --- 核心記錄：記錄真實軌跡 ---
                # 在主迴圈內，確保記錄的是真實 FK 值
                if i in [1, 4]:
                    curr_pos = compute_fk_standalone(thetas, h)
                    # 關鍵：減去該腿的 offset，讓軌跡只剩下「運動範圍」
                    # 假設 leg_offsets[i-1][0] 是 y 偏移, leg_offsets[i-1][2] 是 z 偏移
                    y_rel = (curr_pos[0] * 1000.0) - (leg_offsets[i-1][0] * 1000.0)
                    z_rel = (curr_pos[1] * 1000.0) - (leg_offsets[i-1][2] * 1000.0)
                    log_data.extend([y_rel, z_rel])

            pub.publish(Float64MultiArray(data=[float(t) for t in all_leg_commands]))
            writer.writerow([elapsed] + log_data)
            time.sleep(0.005)

    except KeyboardInterrupt: pass
    finally:
        csv_file.close()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()