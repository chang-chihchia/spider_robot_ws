import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def main():
    csv_path = '/home/chia/spider_ws/end_effector_trajectory_clean.csv'
    df = pd.read_csv(csv_path)
    df = df[(df['time'] >= 6.0) & (df['time'] <= 12.0)]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    stride_points = int(2.0 / 0.005) # 週期長度
    colors = {'leg1': '#B22222', 'leg4': '#00008B'}

    for ax, y_col, z_col, title in zip(axes, ['L1_y', 'L4_y'], ['L1_z', 'L4_z'], ['leg1', 'leg4']):
        # 關鍵：減去平均值，強迫軌跡回到 (0,0) 中心
        y_data = df[y_col] - df[y_col].mean()
        z_data = df[z_col]
        
        # 疊加繪圖
        for i in range(0, len(df) - stride_points, stride_points):
            y_cycle = y_data.iloc[i : i + stride_points]
            z_cycle = z_data.iloc[i : i + stride_points]
            ax.plot(y_cycle, z_cycle, color=colors[title], alpha=0.3, linewidth=1.2)
        
        ax.set_title(title, fontsize=16)
        ax.set_aspect('equal', 'box') # 保持長寬比 1:1，D型才會出來
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.set_xlabel('Relative y / mm')
        ax.set_ylabel('z / mm')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()