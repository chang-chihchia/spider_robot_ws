import pandas as pd
import matplotlib.pyplot as plt

def main():
    df = pd.read_csv('/home/chia/spider_ws/end_effector_trajectory_clean.csv')
    df = df[(df['time'] >= 6.0) & (df['time'] <= 12.0)].copy()

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    stride_points = int(2.0 / 0.005) 

    for ax, y_col, z_col, title, color in zip(axes, ['L1_y', 'L4_y'], ['L1_z', 'L4_z'], ['leg1', 'leg4'], ['#B22222', '#00008B']):
        # --- 核心：數據歸零處理 ---
        # 將該腳位所有數據減去平均值，強迫軌跡回到中心
        y_centered = df[y_col] - df[y_col].mean()
        z_centered = df[z_col] - df[z_col].mean()
        
        # 疊加繪製各週期
        for i in range(0, len(df) - stride_points, stride_points):
            y_cycle = y_centered.iloc[i : i + stride_points]
            z_cycle = z_centered.iloc[i : i + stride_points]
            ax.plot(y_cycle, z_cycle, color=color, alpha=0.3, linewidth=1.2)
        
        ax.set_title(title, fontsize=16)
        # 強制比例與範圍，模擬論文風格
        ax.set_aspect('equal', 'box')
        ax.grid(True, linestyle='--')
        ax.set_xlabel('Relative Y (mm)')
        ax.set_ylabel('Relative Z (mm)')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()