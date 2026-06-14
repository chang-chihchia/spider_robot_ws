主要檔案都在spider_ws\src\spider_bot裡面

------------------------每次初始步驟--------------------------

# 切入到總檔案資料夾
cd ~/spider_ws

# 編譯
colcon build --symlink-install
source install/setup.bash

---------------------開啟gazebo模擬世界-----------------------

# 模擬環境
ros2 launch spider_bot gazebo_launch.py

[(裡面可以自由切換模型)
1.簡化六足模型 spider.urdf
2.履帶模型 spider_tank.urdf]

ros2 launch spider_bot gazebo_new_launch.py(搭配的是spider_bot.urdf)

-------------------------測試啟動檔----------------------------

# 力矩控制測試檔案
ros2 run spider_bot test_node

# 走路控制檔
ros2 run spider_bot run

# 站立測試控制檔
ros2 run spider_bot standup

# 走路控制檔
ros2 run spider_bot run

# 走路控制檔
ros2 run spider_bot tank(使用spider_tank.urdf模型)

---------------------------檔案用途-----------------------------

#控制演算法
spider_ws\src\spider_bot\spider_bot\SpiderBotLib.py

#驅動馬達程式
spider_ws\src\spider_bot\spider_bot\SpiderBotDriver.py

#機器人參數設定
spider_ws\src\spider_bot\config\spider_bot_controllers.yaml(搭配spider.urdf)
spider_ws\src\spider_bot\config\spider_bot_new_controllers.yaml(搭配spider_bot.urdf)

-------------------------驗證數據與圖---------------------------

# xyz、pitch、roll在行走下的實際變化曲線
檔名:五軸測量數據與圖

# 在平地與崎嶇地形行走下的受力曲線
檔名:六條腿受力分析

# 在崎嶇地形行走下的各關節力矩
檔名:腿1、4力矩分析
