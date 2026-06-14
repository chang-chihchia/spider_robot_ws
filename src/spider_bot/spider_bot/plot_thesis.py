import pandas as pd
import matplotlib.pyplot as plt
import os

# 讀取數據
file_path = os.path.expanduser('~/spider_ws/six_leg_effort_data.csv')
df = pd.read_csv(file_path)

# 設定畫布
plt.figure(figsize=(12, 6))

# 迴圈繪製 6 條腿的曲線
for i in range(1, 7):
    plt.plot(df['time'], df[f't{i}'], label=f'Leg {i} Effort', linewidth=1.2)

# 學術圖表設定
plt.title('Spider Robot Gait Analysis: Total Joint Torque Effort', fontsize=14, fontweight='bold')
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Total Joint Torque (Nm)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.5)
plt.legend(loc='upper right', ncol=2) # ncol=2 讓圖例更整齊

plt.tight_layout()
plt.savefig('six_leg_effort_analysis.png', dpi=300)
plt.show()