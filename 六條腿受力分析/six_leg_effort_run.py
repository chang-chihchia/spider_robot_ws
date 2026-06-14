import rclpy
import math
import time
import numpy as np
import csv
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# 修正語法錯誤：將 import 分成兩行
from spider_bot.SpiderBotDriver import SpiderBotDriver
from spider_bot.SpiderBotLib import SpiderBotLib

def main(args=None):
    h = SpiderBotLib()
    a = SpiderBotDriver()   

    if not rclpy.ok():
        rclpy.init()

    ros_node = rclpy.create_node('spider_walk_full_log')

    # =========================================================
    # 數據記錄初始化 (CSV)
    # =========================================================
    # 確保寫入路徑正確，建議使用絕對路徑以避免權限問題
    csv_file = open('six_leg_effort_data.csv', 'w', newline='')
    writer = csv.writer(csv_file)
    writer.writerow(['time', 't1', 't2', 't3', 't4', 't5', 't6'])

    leg_offsets = [[0.1, 0.1, 0], [-0.1, 0.1, 0], [-0.15, 0, 0], [-0.1, -0.1, 0], [0.1, -0.1, 0], [0.15, 0, 0]]
    leg_angles = [45.0, 135.0, 180.0, 225.0, 315.0, 0.0]
    TARGET_Z, EXTEND_R, stride_period, step_length, step_height, PUSH_FORCE_X = -0.08, 0.23, 2.0, 0.1, 0.07, 25.0

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

            all_torques = []
            leg_effort_summary = []

            for i in range(1, 7):
                gamma = np.radians(leg_angles[i - 1])
                group_offset = 0.5 if i in [2, 4, 6] else 0.0
                leg_phase = (phase + group_offset) % 1.0
                is_stance = leg_phase >= 0.5
                
                # 控制運算
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
                    tau += h.force_to_torque(f_vector_local, thetas[0], thetas[1], thetas[2])
                
                tau = np.clip(tau, -7.0, 7.0)
                all_torques.extend(tau)
                leg_effort_summary.append(float(sum(np.abs(tau))))

            pub.publish(Float64MultiArray(data=[float(t) for t in all_torques]))
            writer.writerow([elapsed] + leg_effort_summary)
            
            time.sleep(0.005)

    except KeyboardInterrupt:
        pass
    finally:
        csv_file.close()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()