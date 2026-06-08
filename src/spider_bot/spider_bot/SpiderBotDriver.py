import time
import numpy as np

# 嘗試匯入 Dynamixel SDK。如果未安裝，在純模擬模式下仍可執行。
try:
    from dynamixel_sdk import *
except ImportError:
    # 定義基本常數以防實體通訊報錯
    COMM_SUCCESS = 0
    print("Warning: dynamixel_sdk not found. Physical hardware mode will not work.")

### Dynamixel Addresses ###
ADDR_MODEL_NUMBER                = 0
ADDR_DRIVE_MODE                  = 10
ADDR_OPERATING_MODE              = 11
ADDR_CURRENT_LIMIT               = 38
ADDR_ACCELERATION_LIMIT          = 40
ADDR_VELOCITY_LIMIT              = 44
ADDR_TORQUE_ENABLE               = 64    
ADDR_POSITION_D_GAIN             = 80
ADDR_POSITION_I_GAIN             = 82
ADDR_POSITION_P_GAIN             = 84
ADDR_FEEDFORWARD_2nd_GAIN        = 88
ADDR_FEEDFORWARD_1st_GAIN        = 90
ADDR_GOAL_CURRENT                = 102
ADDR_GOAL_VELOCITY               = 104
ADDR_PROFILE_ACCELERATION        = 108      # VELOCITY BASED PROFILE
ADDR_PROFILE_VELOCITY            = 112      # VELOCITY BASED PROFILE
ADDR_PROFILE_ACCELERATION_TIME   = 108      # TIME BASED PROFILE
ADDR_PROFILE_TIME_SPAN           = 112      # TIME BASED PROFILE
ADDR_GOAL_POSITION               = 116
ADDR_MOVING                      = 122
ADDR_MOVING_STATUS               = 123
ADDR_PRESENT_CURRENT             = 126 
ADDR_PRESENT_POSITION            = 132
ADDR_PRESENT_VELOCITY            = 128      # 速度地址

### Data Byte Length ###
LEN_GOAL_POSITION                = 4
LEN_PRESENT_POSITION             = 4
LEN_GOAL_CURRENT                 = 2
LEN_PRESENT_CURRENT              = 2
LEN_PRESENT_VELOCITY             = 4
LEN_POS_TIME                     = 12
LEN_MOVING                       = 1
LEN_TORQUE_ENABLE                = 1    
LEN_CUR_VEL_POS                  = 10

### Operating Mode Number ###
CURRENT_CONTROL                  = 0
POSITION_CONTROL                 = 3        # Default
CURRENT_BASED_POSITION_CONTROL   = 5

### Velocity profile ###
TIME_BASED                       = 4
VELOCITY_BASED                   = 0

# Protocol version
PROTOCOL_VERSION = 2.0               

BAUDRATE = 1000000 
DEVICENAME = '/dev/u2d2' 
                     
TORQUE_ENABLE  = 1  
TORQUE_DISABLE = 0     

CUR_UNIT = 2.69 # 乘上讀取值可得 mA

# --- ROS 2 與模擬模式所需的匯入 ---
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 (rclpy) not found. Simulation mode will not work.")

class SpiderBotDriver:

    def __init__(self, sim_mode=True):
        self.sim_mode = sim_mode
        
        ## Servo Parameters ##
        self.DriveMode = 0

        self.P_pose_Gain = 1000    # Default: 800
        self.I_pose_Gain = 0       # Default: 0
        self.D_pose_Gain = 4000    # Default: 4700

        self.P_current_Gain = 500  # Default: 800
        self.I_current_Gain = 0    # Default: 0
        self.D_current_Gain = 4000 # Default: 4700

        self.goal_current = 300

        self.TF = 50
        self.TA = self.TF // 3

        self.min_deg = -180.0
        self.max_deg = 180.0
        self.min_bit = 0
        self.max_bit = 4095

        # 初始化 18 顆馬達的各類狀態字典
        self.joint_deg_cmd = {i+1: 0.0 for i in range(18)}
        self.joint_moving = {i+1: 0 for i in range(18)}
        self.joint_position = {i+1: 0.0 for i in range(18)}
        self.joint_torque_enable = {i+1: 0 for i in range(18)}
        
        # 核心狀態反饋結構：包含電流(mA)、角度(deg)、角速度(deg/s)
        self.joint_cur_pos = {i+1: {'cur': 0.0, 'pos': 0.0, 'vel': 0.0} for i in range(18)}

        self.total_servo = len(self.joint_deg_cmd)
        self.servo_mode = 3 # 預設位置控制
        
        # 用於儲存模擬器的最新訂閱數據
        self.latest_joint_state = None

        if not self.sim_mode:
            # ----------------------------------------
            # 1. 實體機器人硬體初始化
            # ----------------------------------------
            if 'rclpy' in globals() and not rclpy.ok(): 
                rclpy.init()
            if 'rclpy' in globals():
                self.node = rclpy.create_node('spider_driver_internal')
            
            self.portHandler = PortHandler(DEVICENAME)
            self.packetHandler = PacketHandler(PROTOCOL_VERSION)
            print("Protocol Version {}".format(self.packetHandler.getProtocolVersion()))
            
            if self.portHandler.openPort():
                print("實體串口開啟成功")
            else:
                print("實體串口開啟失敗"); quit()
                
            if self.portHandler.setBaudRate(BAUDRATE):
                print("波特率設置成功")
            else:
                print("波特率設置失敗"); quit()
            
            # 初始化所有 GroupSyncWrite / GroupSyncRead 實例
            self.groupSyncWritePosition = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
            self.groupSyncReadPosition = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            self.groupSyncReadMoving = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_MOVING, LEN_MOVING)
            self.groupSyncReadTorqueEnable = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_MOVING, LEN_TORQUE_ENABLE)
            self.groupSyncReadCurVelPos = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_CURRENT, LEN_CUR_VEL_POS)
            self.groupSyncWritePositionInTime = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_PROFILE_ACCELERATION, LEN_POS_TIME)
            self.groupSyncWriteCurrent = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT)

            # 初始先安全關閉 Torque
            self.TorqueOff()

            # 向所有馬達註冊群組同步讀寫參數，並設定初使暫存器模式
            for i in range(self.total_servo):
                _id = i + 1
                self.packetHandler.reboot(self.portHandler, _id)
                self.AddParamSyncReadMoving(_id)
                self.AddParamSyncReadPosition(_id)
                self.AddParamSyncTorqueEnable(_id)
                self.AddParamSyncReadCurVelPos(_id)

                if self.servo_mode == 3:
                    self.SetOperatingMode(_id, POSITION_CONTROL)
                    self.SetDrivingMode(_id, TIME_BASED)
                    self.SetTimeBaseProfile(_id, self.TF, self.TA)
                    self.SetPID(_id, self.P_pose_Gain, self.I_pose_Gain, self.D_pose_Gain)
                elif self.servo_mode == 5:
                    self.SetOperatingMode(_id, CURRENT_BASED_POSITION_CONTROL)
                    self.SetGoalCurrent(_id, self.goal_current)
                    self.SetDrivingMode(_id, TIME_BASED)
                    self.SetTimeBaseProfile(_id, self.TF, self.TA)
                    self.SetPID(_id, self.P_current_Gain, self.I_current_Gain, self.D_current_Gain)
                        
            print("Physical Robot Mode: Initialized Successfully")
                    
        else:
            # ----------------------------------------
            # 2. 模擬器環境初始化 (Isaac Sim / Gazebo)
            # ----------------------------------------
            if not ROS2_AVAILABLE:
                print("Error: rclpy not found!"); quit()
            if not rclpy.ok(): 
                rclpy.init()
            self.node = rclpy.create_node('spider_driver_sim')
            # 發送指令到模擬器的 Topic
            self.joint_pub = self.node.create_publisher(Float64MultiArray, '/joint_commands', 10)
            # 接收模擬器真實狀態的反饋 Topic
            self.joint_sub = self.node.create_subscription(JointState, '/joint_states', self._sim_joint_callback, 10)
            print("Simulation Mode: Isaac Sim Bridge Ready")

    def _sim_joint_callback(self, msg):
        """ 模擬模式回呼函數：自動解析並儲存來自 Isaac Sim 的關節數據 """
        self.latest_joint_state = msg

    ##############################################
    ### Torque & Current Control Section (下發) ###
    ##############################################
    def SetGoalCurrent_Direct(self, ID, current_mA):
        """ 單一馬達寫入電流 (用於個別測試) """
        if self.sim_mode: 
            return

        raw_current = int(current_mA / CUR_UNIT) 
        raw_current = max(min(raw_current, 2047), -2047) # 安全飽和限制幅
        
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, ADDR_GOAL_CURRENT, raw_current)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{ID}] Direct Current Write Failed")

    def SetTorqueByLeg(self, leg_no, torque_vector_Nm):
        """
        根據高階 Lib 算出的力矩向量控制單一腿 (包含3顆馬達)
        torque_vector_Nm: np.array([tau1, tau2, tau3])
        """
        leg_id_map = {1:1, 2:4, 3:7, 4:10, 5:13, 6:16}
        start_id = leg_id_map.get(leg_no)
        if start_id is None:
            print(f"Error: Invalid Leg Number {leg_no}")
            return
        
        mA_per_Nm = 750.0 # 根據 XM430 規格書估算 (1 Nm 約等於 750 mA)
        
        if not self.sim_mode:
            for i in range(3):
                servo_id = start_id + i
                target_mA = torque_vector_Nm[i] * mA_per_Nm
                self.SetGoalCurrent_Direct(servo_id, target_mA)
        else:
            # 模擬模式下建議直接使用 SyncWriteAllCurrents 做全機同步
            pass

    def SyncWriteAllCurrents(self, all_mA_list):
        """
        全機高頻同步寫入指令：
        - 實體模式：以 SyncWrite 一次性將各馬達的電流寫入暫存器
        - 模擬模式：將電流或力矩列表打包為 Float64MultiArray 發布給 Isaac Sim
        """
        if not self.sim_mode:
            # --- 實體模式：同步寫入電流 ---
            for i in range(18):
                servo_id = i + 1
                raw_val = int(all_mA_list[i] / CUR_UNIT)
                raw_val = max(min(raw_val, 2047), -2047)
                
                param_goal_current = [DXL_LOBYTE(raw_val), DXL_HIBYTE(raw_val)]
                self.groupSyncWriteCurrent.addParam(servo_id, param_goal_current)
                
            self.groupSyncWriteCurrent.txPacket()
            self.groupSyncWriteCurrent.clearParam()
        else:
            # --- 模擬模式：發送 Topic 給 Isaac Sim ---
            if ROS2_AVAILABLE:
                msg = Float64MultiArray()
                msg.data = [float(c) for c in all_mA_list]
                self.joint_pub.publish(msg)

    def SyncWriteAllPositions(self, all_deg_list):
        """
        全機同步位置寫入：
        - 實體模式：將角度轉為 Bit 碼並下發
        - 模擬模式：通常會將角度轉為弧度 (Rad) 後發送至模擬控制
        """
        if not self.sim_mode:
            for i in range(18):
                servo_id = i + 1
                raw_pos = int(self.map(all_deg_list[i], self.min_deg, self.max_deg, self.min_bit, self.max_bit))
                
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(raw_pos)), 
                                       DXL_HIBYTE(DXL_LOWORD(raw_pos)), 
                                       DXL_LOBYTE(DXL_HIWORD(raw_pos)), 
                                       DXL_HIBYTE(DXL_HIWORD(raw_pos))]
                self.groupSyncWritePosition.addParam(servo_id, param_goal_position)
            self.groupSyncWritePosition.txPacket()
            self.groupSyncWritePosition.clearParam()
        else:
            if ROS2_AVAILABLE:
                msg = Float64MultiArray()
                # 轉換為弧度以利模擬器讀取
                msg.data = [float(np.radians(d)) for d in all_deg_list]
                self.joint_pub.publish(msg)

    #######################################
    ### State Feedback Section (反饋) ###
    #######################################
    def UpdateAllStatesSync(self):
        """
        全機狀態同步讀取：
        - 實體模式：透過群組同步讀取，同時獲得 18 顆馬達的電流、速度與位置並轉換為標準物理單位
        - 模擬模式：呼叫 rclpy.spin_once 刷新資料，並將模擬器的弧度、Nm 轉回腳本專用的度與 mA
        """
        if not self.sim_mode:
            # --- 實體模式 ---
            dxl_comm_result = self.groupSyncReadCurVelPos.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS: 
                return
                
            for i in range(18):
                servo_id = i + 1
                if self.groupSyncReadCurVelPos.isAvailable(servo_id, ADDR_PRESENT_CURRENT, LEN_CUR_VEL_POS):
                    raw_cur = self.groupSyncReadCurVelPos.getData(servo_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT)
                    raw_vel = self.groupSyncReadCurVelPos.getData(servo_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
                    raw_pos = self.groupSyncReadCurVelPos.getData(servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                    
                    self.joint_cur_pos[servo_id]['cur'] = self.convert_to_mA(raw_cur)
                    self.joint_cur_pos[servo_id]['vel'] = self.convert_to_deg_sec(raw_vel)
                    self.joint_cur_pos[servo_id]['pos'] = self.map(raw_pos, self.min_bit, self.max_bit, self.min_deg, self.max_deg)
        else:
            # --- 模擬模式 ---
            if ROS2_AVAILABLE:
                # 觸發 ROS 2 佇列事件以更新最新回呼狀態
                rclpy.spin_once(self.node, timeout_sec=0.001)
                
                if self.latest_joint_state is not None:
                    NM_TO_MA = 750.0 
                    state_map = {name: (pos, vel, eff) for name, pos, vel, eff in zip(
                        self.latest_joint_state.name, 
                        self.latest_joint_state.position, 
                        self.latest_joint_state.velocity,
                        self.latest_joint_state.effort)}
                    
                    for i in range(18):
                        servo_id = i + 1
                        joint_name = f'joint{servo_id}'
                        
                        if joint_name in state_map:
                            pos_rad, vel_rad, effort_nm = state_map[joint_name]
                            # 將模擬器的國際標準單位轉為機器人傳統度數與電流單位
                            self.joint_cur_pos[servo_id]['pos'] = np.degrees(pos_rad)
                            self.joint_cur_pos[servo_id]['vel'] = np.degrees(vel_rad)
                            self.joint_cur_pos[servo_id]['cur'] = effort_nm * NM_TO_MA

    def ReadMoving(self):
        """ 讀取移動中標誌 """
        if self.sim_mode:
            for i in range(self.total_servo):
                self.joint_moving[i+1] = 0
            return

        dxl_comm_result = self.groupSyncReadMoving.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("ReadMoving Fail: {:}".format(self.packetHandler.getTxRxResult(dxl_comm_result)))

        for i in range(self.total_servo):
            servo_id = i + 1
            if self.groupSyncReadMoving.isAvailable(servo_id, ADDR_MOVING, LEN_MOVING):
                self.joint_moving[servo_id] = self.groupSyncReadMoving.getData(servo_id, ADDR_MOVING, LEN_MOVING)

    ######################################
    ### Math & Data Convert Functions ###
    ######################################
    def map(self, val, in_min, in_max, out_min, out_max):   
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def convert_to_mA(self, raw):
        # 處理 2 的補數負數 (2 Bytes)
        if raw > 32767: 
            raw -= 65536
        return raw * 2.69

    def convert_to_deg_sec(self, raw):
        # 處理 4 Bytes 速度負數
        if raw > 2147483647: 
            raw -= 4294967296
        return raw * 0.229 * 6.0  # 0.229 rpm 轉為 deg/sec

    #######################################
    ### Hardware Setup & Sync Functions ###
    #######################################
    def AddParamSyncReadMoving(self, ID):
        self.groupSyncReadMoving.addParam(ID)

    def AddParamSyncReadPosition(self, ID):
        self.groupSyncReadPosition.addParam(ID)

    def AddParamSyncTorqueEnable(self, ID):
        self.groupSyncReadTorqueEnable.addParam(ID)

    def AddParamSyncReadCurVelPos(self, ID):
        self.groupSyncReadCurVelPos.addParam(ID)

    def SetOperatingMode(self, ID, MODE):
        if self.sim_mode: 
            return
        
        # 切換模式前須先關閉扭力
        self.packetHandler.write1ByteTxRx(self.portHandler, ID, ADDR_TORQUE_ENABLE, 0)
        self.packetHandler.write1ByteTxRx(self.portHandler, ID, ADDR_OPERATING_MODE, MODE)
        self.packetHandler.write1ByteTxRx(self.portHandler, ID, ADDR_TORQUE_ENABLE, 1)

        present_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, ADDR_OPERATING_MODE)
        if dxl_comm_result == COMM_SUCCESS:
            mode_names = {0: "Torque Control", 3: "Position Control", 5: "Current-based Position Control"}
            print(f"Motor {ID} Operating Mode is {mode_names.get(present_mode, 'Unknown Mode')}")

    def SetOperatingMode_byLeg(self, leg_no, MODE):
        if self.sim_mode: 
            return      
        leg_id_map = {1:1, 2:4, 3:7, 4:10, 5:13, 6:16}
        start_id = leg_id_map.get(leg_no, 1)
        for i in range(3):
            self.SetOperatingMode(start_id + i, MODE)

    def SetDrivingMode(self, ID, Base):
        if self.sim_mode: 
            return
        self.packetHandler.write1ByteTxRx(self.portHandler, ID, ADDR_DRIVE_MODE, Base)
        self.DriveMode, _, _ = self.packetHandler.read1ByteTxRx(self.portHandler, ID, ADDR_DRIVE_MODE)

    def SetDrivingMode_byLeg(self, leg_no):
        if self.sim_mode: 
            return      
        leg_id_map = {1:1, 2:4, 3:7, 4:10, 5:13, 6:16}
        start_id = leg_id_map.get(leg_no, 1)
        for i in range(3):
            self.SetDrivingMode(start_id + i, TIME_BASED)

    def SetPID(self, ID, set_P_Gain, set_I_Gain, set_D_Gain):
        if self.sim_mode: 
            return
        self.packetHandler.write2ByteTxRx(self.portHandler, ID, ADDR_POSITION_P_GAIN, set_P_Gain)
        self.packetHandler.write2ByteTxRx(self.portHandler, ID, ADDR_POSITION_I_GAIN, set_I_Gain)
        self.packetHandler.write2ByteTxRx(self.portHandler, ID, ADDR_POSITION_D_GAIN, set_D_Gain)

    def SetPID_byLeg(self, leg_no, set_P_Gain, set_I_Gain, set_D_Gain):
        if self.sim_mode: 
            return
        leg_id_map = {1:1, 2:4, 3:7, 4:10, 5:13, 6:16}
        start_id = leg_id_map.get(leg_no, 1)
        for i in range(3):
            self.SetPID(start_id + i, set_P_Gain, set_I_Gain, set_D_Gain)

    def SetTimeBaseProfile(self, ID, set_Tf, set_Ta):
        if self.sim_mode: 
            return
        self.packetHandler.write4ByteTxRx(self.portHandler, ID, ADDR_PROFILE_ACCELERATION_TIME, int(set_Ta))
        self.packetHandler.write4ByteTxRx(self.portHandler, ID, ADDR_PROFILE_TIME_SPAN, int(set_Tf))

    def SetGoalCurrent(self, ID, SetCur):
        if self.sim_mode: 
            return
        self.packetHandler.write2ByteTxRx(self.portHandler, ID, ADDR_GOAL_CURRENT, SetCur)

    def SetGoalCurrent_byLeg(self, leg_no, joint1_cur, joint2_cur, joint3_cur):
        if self.sim_mode: 
            return
        leg_id_map = {1:1, 2:4, 3:7, 4:10, 5:13, 6:16}
        start_id = leg_id_map.get(leg_no, 1)
        curs = [joint1_cur, joint2_cur, joint3_cur]
        for i in range(3):
            self.SetGoalCurrent(start_id + i, curs[i])

    def TorqueOn(self):
        if self.sim_mode: 
            print("SIM_MODE: Torque Enabled (Virtual)")
            return
        for i in range(self.total_servo):
            self.packetHandler.write1ByteTxRx(self.portHandler, i+1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def TorqueOff(self):
        if self.sim_mode: 
            print("SIM_MODE: Torque Disabled (Virtual)")
            return
        for i in range(self.total_servo):
            self.packetHandler.write1ByteTxRx(self.portHandler, i+1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def LegTorqueOn(self, leg_no):
        if self.sim_mode: 
            return
        leg_id_map = {1:1, 2:4, 3:7, 4:10, 5:13, 6:16}
        start_id = leg_id_map.get(leg_no, 1)
        for i in range(3):
            self.packetHandler.write1ByteTxRx(self.portHandler, start_id + i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def LegTorqueOff(self, leg_no):
        if self.sim_mode: 
            return
        leg_id_map = {1:1, 2:4, 3:7, 4:10, 5:13, 6:16}
        start_id = leg_id_map.get(leg_no, 1)
        for i in range(3):
            self.packetHandler.write1ByteTxRx(self.portHandler, start_id + i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)