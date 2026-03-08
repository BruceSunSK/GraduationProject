#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test Local Planning Data Visualization
Reads vehicle_data.csv and planning_times.csv, then plots:
1. XY trajectory with velocity-colored continuous path
2. Velocity vs time
3. Ax vs Ay scatter
4. Ax vs time
5. Curvature vs time
6. Planning time vs index
7. Terrain distance vs time
"""

import os
import sys
import glob
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.collections import LineCollection
from mpl_toolkits.axes_grid1 import make_axes_locatable


DOWNSAMPLE_STEP = 5


def find_latest_folder(base_dir):
    """返回 base_dir 下最新修改的文件夹（按修改时间）"""
    folders = glob.glob(os.path.join(base_dir, "????????_??????"))
    if not folders:
        return None
    return max(folders, key=os.path.getmtime)


def plot_xy_with_velocity(df, out_dir):
    """绘制 XY 轨迹，用彩色连续线条表示速度（条带效果）"""
    fig, ax = plt.subplots(figsize=(8, 8))

    x = df['x'].values
    y = df['y'].values
    v = df['v'].values

    # 构建线段集合
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    # 使用每个线段起始点的速度作为颜色
    norm = plt.Normalize(v.min(), v.max())
    lc = LineCollection(segments, cmap='jet', norm=norm)
    lc.set_array(v[:-1])
    lc.set_linewidth(4)  # 加粗线条形成条带效果
    ax.add_collection(lc)
    ax.autoscale()
    ax.set_aspect('equal')

    # 颜色条（使用make_axes_locatable使高度与axes相同）
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    cbar = plt.colorbar(lc, cax=cax)
    cbar.set_label('Velocity (m/s)')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Vehicle Trajectory with Velocity')
    ax.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'xy_path.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"XY path plot saved to {out_dir}")


def plot_velocity(df, out_dir):
    """绘制速度-时间图"""
    fig, ax = plt.subplots(figsize=(8, 4))

    t = df['timestamp'].values - df['timestamp'].iloc[0]
    v = df['v'].values

    ax.plot(t, v, 'b-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Velocity Profile')
    ax.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'velocity.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Velocity plot saved to {out_dir}")


def plot_ax_vs_ay(df, out_dir):
    """绘制加速度散点图（ax vs ay），增大点尺寸"""
    fig, ax = plt.subplots(figsize=(6, 6))

    ax.scatter(df['ay'], df['ax'], c='b', s=40, alpha=0.6, edgecolors='none')

    # 对称坐标轴
    max_abs = max(np.abs(df['ax']).max(), np.abs(df['ay']).max()) * 1.05
    ax.set_xlim(-max_abs, max_abs)
    ax.set_ylim(-max_abs, max_abs)
    ax.set_aspect('equal')

    ax.set_xlabel('ay (m/s²)')
    ax.set_ylabel('ax (m/s²)')
    ax.set_title('Acceleration Distribution')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.axhline(0, color='gray', linewidth=0.5)
    ax.axvline(0, color='gray', linewidth=0.5)

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'acceleration_scatter.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Acceleration scatter plot saved to {out_dir}")


def plot_ax_t(df, out_dir):
    """绘制纵向加速度-时间图"""
    fig, ax = plt.subplots(figsize=(8, 4))

    t = df['timestamp'].values - df['timestamp'].iloc[0]
    ax_val = df['ax'].values

    ax.plot(t, ax_val, 'm-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('ax (m/s²)')
    ax.set_title('Longitudinal Acceleration Profile')
    ax.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'ax_t.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Ax-t plot saved to {out_dir}")


def plot_curvature(df, out_dir):
    """绘制曲率-时间图，y轴关于0对称"""
    fig, ax = plt.subplots(figsize=(8, 4))

    t = df['timestamp'].values - df['timestamp'].iloc[0]
    kappa = df['kappa'].values

    ax.plot(t, kappa, 'g-', linewidth=2)

    # 对称y轴
    max_abs = np.abs(kappa).max() * 1.05
    ax.set_ylim(-max_abs, max_abs)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Curvature (1/m)')
    ax.set_title('Curvature Profile')
    ax.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'curvature.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Curvature plot saved to {out_dir}")


def plot_planning_times(df_times, out_dir):
    """绘制规划耗时-索引图，并标注均值和最大值"""
    fig, ax = plt.subplots(figsize=(8, 4))

    idx = df_times['index'].values
    cost = df_times['cost_time_ms'].values

    ax.plot(idx, cost, 'c-', linewidth=2, label='Planning time')
    # ax.set_ylim(0, 100)   # 固定范围 0~100 ms
    ax.set_ylim(0, 60)   # 固定范围 0~100 ms
    mean_val = cost.mean()
    max_val = cost.max()
    ax.axhline(mean_val, color='orange', linestyle='--', linewidth=2, label=f'Mean: {mean_val:.1f} ms')
    ax.axhline(max_val, color='red', linestyle='--', linewidth=2, label=f'Max: {max_val:.1f} ms')

    ax.set_xlabel('Planning Index')
    ax.set_ylabel('Cost Time (ms)')
    ax.set_title('Planning Time per Cycle')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'planning_times.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Planning times plot saved to {out_dir}")


def plot_terrain_distance(df, out_dir):
    fig, ax = plt.subplots(figsize=(8, 4))
    t = df['timestamp'].values - df['timestamp'].iloc[0]
    dist = df['terrain_distance'].values
    ax.plot(t, dist, color='orange', linewidth=2, label='Terrain distance')
    # y轴从0开始，上限为数据最大值加5%余量
    y_max = dist.max()
    ax.set_ylim(bottom=0, top=y_max * 1.05 if y_max > 0 else 1.0)
    min_val = dist.min()
    ax.axhline(y=min_val, color='gray', linestyle='--', linewidth=2,
               label=f'Min: {min_val:.2f} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance to terrain (m)')
    ax.set_title('Terrain Distance Profile')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.legend(loc='upper right')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'terrain_distance.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Terrain distance plot saved to {out_dir}")


def main():
    parser = argparse.ArgumentParser(description='Plot test local planning results.')
    parser.add_argument('folder', nargs='?', help='Path to the folder containing CSV files (optional, uses latest if not provided)')
    args = parser.parse_args()

    if args.folder:
        folder = args.folder
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base_dir = os.path.abspath(os.path.join(script_dir, '../result/test/local_planning/'))
        if not os.path.exists(base_dir):
            print(f"Base directory not found: {base_dir}")
            sys.exit(1)
        folder = find_latest_folder(base_dir)
        if folder is None:
            print("No data folders found in", base_dir)
            sys.exit(1)
        print(f"Using latest folder: {folder}")

    # 读取车辆数据
    vehicle_csv = os.path.join(folder, 'vehicle_data.csv')
    if not os.path.exists(vehicle_csv):
        print(f"vehicle_data.csv not found in {folder}")
        sys.exit(1)
    df_vehicle = pd.read_csv(vehicle_csv).iloc[::DOWNSAMPLE_STEP]
    required_cols = ['timestamp', 'x', 'y', 'theta', 'v', 'ax', 'ay', 'kappa', 'terrain_distance']
    for col in required_cols:
        if col not in df_vehicle.columns:
            print(f"Missing column: {col} in vehicle_data.csv")
            sys.exit(1)

    # 读取规划耗时数据
    times_csv = os.path.join(folder, 'planning_times.csv')
    if not os.path.exists(times_csv):
        print(f"planning_times.csv not found in {folder}")
        sys.exit(1)
    df_times = pd.read_csv(times_csv).iloc[::DOWNSAMPLE_STEP]
    required_time_cols = ['index', 'cost_time_ms']
    for col in required_time_cols:
        if col not in df_times.columns:
            print(f"Missing column: {col} in planning_times.csv")
            sys.exit(1)

    # 保存独立图片
    plot_xy_with_velocity(df_vehicle, folder)
    plot_velocity(df_vehicle, folder)
    plot_ax_vs_ay(df_vehicle, folder)
    plot_ax_t(df_vehicle, folder)
    plot_curvature(df_vehicle, folder)
    plot_planning_times(df_times, folder)
    plot_terrain_distance(df_vehicle, folder)

    # 组合图（3行3列，顺序：1,2,4,3,5,6,7）
    fig, axes = plt.subplots(3, 3, figsize=(18, 18))

     # 图1: XY轨迹（保持不变，放在 (0,0)）
    ax1 = axes[0, 0]
    x = df_vehicle['x'].values
    y = df_vehicle['y'].values
    v = df_vehicle['v'].values
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm = plt.Normalize(v.min(), v.max())
    lc = LineCollection(segments, cmap='jet', norm=norm)
    lc.set_array(v[:-1])
    lc.set_linewidth(4)
    ax1.add_collection(lc)
    ax1.autoscale()
    ax1.set_aspect('equal')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Trajectory with Velocity')
    ax1.grid(True, linestyle='--', alpha=0.7)
    cbar = fig.colorbar(lc, ax=ax1, fraction=0.05, pad=0.05)
    cbar.set_label('Velocity (m/s)')

    # 图2: v-t (0,1)
    ax2 = axes[0, 1]
    t = df_vehicle['timestamp'] - df_vehicle['timestamp'].iloc[0]
    ax2.plot(t, df_vehicle['v'], 'b-', linewidth=2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Velocity')
    ax2.grid(True, linestyle='--', alpha=0.7)

    # 图4: ax-t (0,2)
    ax4 = axes[0, 2]
    ax4.plot(t, df_vehicle['ax'], 'm-', linewidth=2)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('ax (m/s²)')
    ax4.set_title('Longitudinal Acceleration')
    ax4.grid(True, linestyle='--', alpha=0.7)

    # 图3: ax-ay散点 (1,0)
    ax3 = axes[1, 0]
    ax3.scatter(df_vehicle['ay'], df_vehicle['ax'], c='b', s=40, alpha=0.6, edgecolors='none')
    max_abs = max(np.abs(df_vehicle['ax']).max(), np.abs(df_vehicle['ay']).max()) * 1.05
    ax3.set_xlim(-max_abs, max_abs)
    ax3.set_ylim(-max_abs, max_abs)
    ax3.set_aspect('equal')
    ax3.set_xlabel('ay (m/s²)')
    ax3.set_ylabel('ax (m/s²)')
    ax3.set_title('Acceleration')
    ax3.grid(True, linestyle='--', alpha=0.7)
    ax3.axhline(0, color='gray', linewidth=0.5)
    ax3.axvline(0, color='gray', linewidth=0.5)

    # 图5: kappa-t (1,1)
    ax5 = axes[1, 1]
    ax5.plot(t, df_vehicle['kappa'], 'g-', linewidth=2)
    max_kappa = np.abs(df_vehicle['kappa']).max() * 1.05
    ax5.set_ylim(-max_kappa, max_kappa)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Curvature (1/m)')
    ax5.set_title('Curvature')
    ax5.grid(True, linestyle='--', alpha=0.7)

    # 图6: planning times (1,2)
    ax6 = axes[1, 2]
    idx = df_times['index'].values
    cost = df_times['cost_time_ms'].values
    ax6.plot(idx, cost, 'c-', linewidth=2, label='time')
    # ax6.set_ylim(0, 100)
    ax6.set_ylim(0, 60)
    mean_val = cost.mean()
    max_val = cost.max()
    ax6.axhline(mean_val, color='orange', linestyle='--', linewidth=2, label=f'Mean: {mean_val:.1f} ms')
    ax6.axhline(max_val, color='red', linestyle='--', linewidth=2, label=f'Max: {max_val:.1f} ms')
    ax6.set_xlabel('Index')
    ax6.set_ylabel('Cost Time (ms)')
    ax6.set_title('Planning Time')
    ax6.grid(True, linestyle='--', alpha=0.7)
    ax6.legend(loc='upper right', fontsize=8)

    # 图7: terrain distance (2,0)
    ax7 = axes[2, 0]
    t = df_vehicle['timestamp'] - df_vehicle['timestamp'].iloc[0]
    dist = df_vehicle['terrain_distance'].values
    ax7.plot(t, dist, color='orange', linewidth=2, label='Terrain distance')
    y_max = dist.max()
    ax7.set_ylim(bottom=0, top=y_max * 1.05 if y_max > 0 else 1.0)
    min_val = dist.min()
    ax7.axhline(y=min_val, color='gray', linestyle='--', linewidth=2,
                label=f'Min: {min_val:.2f} m')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Terrain distance (m)')
    ax7.set_title('Terrain Distance')
    ax7.grid(True, linestyle='--', alpha=0.7)
    ax7.legend(loc='upper right')

    # 其余子图（2,1）和（2,2）留空或隐藏
    axes[2, 1].axis('off')
    axes[2, 2].axis('off')

    plt.tight_layout()
    plt.savefig(os.path.join(folder, 'combined.png'), dpi=150, bbox_inches='tight')
    plt.show()
    print(f"Combined plot saved to {folder}")


if __name__ == "__main__":
    main()