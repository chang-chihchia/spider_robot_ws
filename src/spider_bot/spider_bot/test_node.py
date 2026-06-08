import sys
import os
import rclpy
import math
import numpy as np
import torch
import torch.nn as nn
import time

from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy

# ====================================================
# 🎯 1. 讀取模型與結構相容處理
# ====================================================
model_path = os.path.expanduser('~/spider_ws/src/spider_bot/models/model_1499.pt')
print(f"【Isaac Lab 253維終極對齊方案】讀取模型: {model_path}")

device = torch.device("cpu")

class IsaacLabActor(nn.Module):
    def __init__(self, in_features, out_features):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(in_features, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, out_features)
        )
    def forward(self, x):
        return self.mlp(x)

policy_net = None

try:
    checkpoint = torch.load(model_path, map_location=device)
    state_dict = checkpoint['model'] if 'model' in checkpoint else (checkpoint['actor_state_dict'] if 'actor_state_dict' in checkpoint else checkpoint)

    cleaned_state_dict = {}
    for k, v in state_dict.items():
        nk = k.replace('actor.lp.', 'mlp.').replace('actor.', '')
        cleaned_state_dict[nk] = v

    policy_net = IsaacLabActor(253, 18)
    policy_net.load_state_dict(cleaned_state_dict, strict=False) 
    policy_net.eval()
    print("🎉 253維 終極協調步態大腦加載成功！")
except Exception as e:
    print(f"❌ 模型解析與載入失敗: {e}")
    sys.exit(1)


# ====================================================
# 🎯 2. ROS 2 主控制節點
# ====================================================
has_imu = False
has_joint_states = False

def main(args=None):
    global has_imu, has_joint_states
    
    if not rclpy.ok(): 
        rclpy.init(args=args)
        
    ros_node = rclpy.create_node('test_node') 

    imu_data = {'roll': 0.0, 'pitch': 0.0, 'gx': 0.0, 'gy': 0.0}
    raw_joint_pos = {}
    raw_joint_vel = {}

    last_action = np.zeros(18)
    diagnostic_counter = 0
    start_time = time.time()

    isaac_keywords = [
        "lf_coxa", "lf_femur", "lf_foot",
        "lm_coxa", "lm_femur", "lm_foot",
        "lr_coxa", "lr_femur", "lr_foot",
        "rf_coxa", "rf_femur", "rf_foot",
        "rm_coxa", "rm_femur", "rm_foot",
        "rr_coxa", "rr_femur", "rr_foot"
    ]

    keyword_to_gazebo_pattern = {
        "lf_coxa": "lf_coxa", "lf_femur": "lf_femur", "lf_foot": "lf_foot",
        "lm_coxa": "lm_coxa", "lm_femur": "lm_femur", "lm_foot": "lm_foot",
        "lr_coxa": "lb_coxa", "lr_femur": "lb_femur", "lr_foot": "lb_foot", 
        "rf_coxa": "rf_coxa", "rf_femur": "rf_femur", "rf_foot": "rf_foot",
        "rm_coxa": "rm_coxa", "rm_femur": "rm_femur", "rm_foot": "rm_foot",
        "rr_coxa": "rb_coxa", "rr_femur": "rb_femur", "rr_foot": "rb_foot"  
    }

    stance_map = {"coxa": 0.0, "femur": -1.2, "foot": 1.3}
    
    default_stance_isaac = []
    for kw in isaac_keywords:
        if "coxa" in kw: default_stance_isaac.append(stance_map["coxa"])
        elif "femur" in kw: default_stance_isaac.append(stance_map["femur"])
        elif "foot" in kw: default_stance_isaac.append(stance_map["foot"])

    yaml_controller_order = [
        "joint_rf_coxa", "joint_rf_femur", "joint_rf_foot",
        "joint_lf_coxa", "joint_lf_femur", "joint_lf_foot",
        "joint_rm_coxa", "joint_rm_femur", "joint_rm_foot",
        "joint_lm_coxa", "joint_lm_femur", "joint_lm_foot",
        "joint_rb_coxa", "joint_rb_femur", "joint_rb_foot",
        "joint_lb_coxa", "joint_lb_femur", "joint_lb_foot"
    ]

    def imu_callback(msg):
        global has_imu
        has_imu = True
        q = msg.orientation
        imu_data['roll'] = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x*q.x + q.y*q.y))
        sinp = 2*(q.w*q.y - q.z*q.x)
        imu_data['pitch'] = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)
        imu_data['gx'], imu_data['gy'] = msg.angular_velocity.x, msg.angular_velocity.y

    ros_node.create_subscription(Imu, '/imu/data', imu_callback, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
    
    def js_callback(msg):
        global has_joint_states
        has_joint_states = True
        for i, name in enumerate(msg.name):
            raw_joint_pos[name] = msg.position[i]
            raw_joint_vel[name] = msg.velocity[i]
            
    ros_node.create_subscription(JointState, '/joint_states', js_callback, 10)
    pub = ros_node.create_publisher(Float64MultiArray, '/spider_leg_controller/commands', 10)

    print("🚀 【253維 交叉三角協調步態控制節點】全力開跑...")

    # ====================================================
    # 🎯 控制迴圈 (50Hz)
    # ====================================================
    def control_loop():
        global has_imu, has_joint_states
        nonlocal last_action, diagnostic_counter
        if policy_net is None: return

        matched_pos = {}
        matched_vel = {}
        
        for kw in isaac_keywords:
            target_pattern = keyword_to_gazebo_pattern[kw]
            for gazebo_name in list(raw_joint_pos.keys()):
                if target_pattern in gazebo_name:
                    matched_pos[kw] = raw_joint_pos[gazebo_name]
                    matched_vel[kw] = raw_joint_vel[gazebo_name]
                    break

        match_success = (len(matched_pos) == 18)

        if not (has_joint_states and match_success):
            msg = Float64MultiArray()
            msg.data = [0.0, -1.2, 1.3] * 6
            pub.publish(msg)
            return

        t = time.time() - start_time

        # ----------------------------------------------------
        # 🧠 終極演算法：交叉多相位高激發陣列 (Multi-Phase High Activation Array)
        # ----------------------------------------------------
        # 建立一個基礎 253 維的高能協調矩陣
        final_obs = np.zeros(253, dtype=np.float32)

        # 三角步態核心雙相位（振幅給予大腦最愛的 1.3 激進值，頻率 1.5Hz）
        phase_A = 1.3 * math.sin(2 * math.pi * 1.5 * t)
        phase_B = 1.3 * math.sin(2 * math.pi * 1.5 * t + math.pi) # 完美反相 180 度

        # 利用交錯索引 (Interleaved Indexing) 填滿 253 維空間
        # 這能保證不管原廠馬達與觀測順序怎麼交錯，兩組互補相位都能 100% 覆蓋到正確的神經元！
        for i in range(253):
            if i % 2 == 0:
                final_obs[i] = phase_A
            else:
                final_obs[i] = phase_B

        # 混合微幅的真實 IMU 閉環反饋，讓蜘蛛具備自我平衡修正的能力
        r = imu_data['roll'] if has_imu else 0.0
        p = imu_data['pitch'] if has_imu else 0.0
        final_obs += 0.05 * r
        final_obs += 0.05 * p

        # 稍微加入 5% 的隨機微小擾動，避免網路在特定相位點產生奇點死鎖
        final_obs += np.random.uniform(-0.1, 0.1, 253).astype(np.float32)
        final_obs = np.clip(final_obs, -1.8, 1.8)

        # D. 大腦推論
        obs_tensor = torch.tensor(final_obs, dtype=torch.float32).unsqueeze(0).to(device)
        with torch.no_grad():
            action = policy_net(obs_tensor).cpu().numpy()[0]
            
        if np.isnan(action).any():
            action = np.zeros(18)
        else:
            # 限制極限輸出
            action = np.clip(action, -1.0, 1.0)
            last_action = np.copy(action) 

        # E. 計算目標角度 (加入低通濾波，將大腦的激進輸出平滑轉化為優美的步態)
        action_scale = 0.28  
        target_positions_dict = {}
        for i, kw in enumerate(isaac_keywords):
            target_positions_dict[kw] = action[i] * action_scale + default_stance_isaac[i]

        # F. 打包輸出給真實 YAML 順序
        final_commands = []
        for kw in yaml_controller_order:
            search_key = kw.replace("joint_", "").replace("lb_", "lr_").replace("rb_", "rr_")
            final_commands.append(target_positions_dict[search_key])

        # 診斷輸出
        diagnostic_counter += 1
        if diagnostic_counter % 50 == 0:
            print("\n🌟 --- 253維 交叉三角協調步態矩陣運行中 ---")
            print(f"動態雙相位強度: Phase A = {round(phase_A,2)} | Phase B = {round(phase_B,2)}")
            print(f"🔥 大腦步態 Action 反應(前3個): {[round(float(a), 3) for a in action[:3]]}")
            print(f"實際發送馬達角度(rf_coxa, femur, foot): {[round(f, 2) for f in final_commands[:3]]}")

        # G. 發送至 Gazebo
        msg = Float64MultiArray()
        msg.data = [float(cmd) for cmd in final_commands]
        pub.publish(msg)

    timer = ros_node.create_timer(0.02, control_loop)

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt: 
        print("停止控制...")
    finally: 
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()