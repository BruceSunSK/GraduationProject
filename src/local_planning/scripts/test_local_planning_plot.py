#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test Local Planning Data Visualization
Reads vehicle_data.csv and plots:
1. XY path with colored rectangles (velocity-based)
2. Velocity vs time
3. Acceleration scatter (ax vs ay)
4. Curvature vs time
"""

import os
import sys
import glob
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm


# 车辆几何参数（可根据实际调整）
VEHICLE_LENGTH = 3.8   # 米
VEHICLE_WIDTH = 2.0    # 米
RECT_SPARSE_STEP = 1   # 每5个点绘制一个矩形（可调）

def find_latest_folder(base_dir):
    """返回 base_dir 下最新修改的文件夹（按修改时间）"""
    folders = glob.glob(os.path.join(base_dir, "????????_??????"))
    if not folders:
        return None
    return max(folders, key=os.path.getmtime)

def estimate_yaw_from_points(x, y):
    """通过差分估算航向角 (弧度)"""
    yaw = np.zeros_like(x)
    # 前向差分
    for i in range(len(x)-1):
        dx = x[i+1] - x[i]
        dy = y[i+1] - y[i]
        yaw[i] = np.arctan2(dy, dx)
    # 最后一个点沿用前一点的航向
    if len(x) > 1:
        yaw[-1] = yaw[-2]
    return yaw

def plot_xy_with_velocity(df, out_dir):
    """绘制 XY 轨迹，用颜色表示速度，并添加车辆矩形（稀疏）"""
    fig, ax = plt.subplots(figsize=(8, 8))

    x = df['x'].values
    y = df['y'].values
    v = df['v'].values

    if 'theta' in df.columns:
        theta = df['theta'].values
    else:
        theta = estimate_yaw_from_points(x, y)

    v_norm = (v - v.min()) / (v.max() - v.min() + 1e-9)
    cmap = cm.jet

    # 绘制背景轨迹线
    ax.plot(x, y, 'k-', linewidth=1, alpha=0.5, label='Path')

    # 稀疏绘制矩形
    for i in range(0, len(x), RECT_SPARSE_STEP):
        cx, cy = x[i], y[i]
        yaw = theta[i]
        half_l = VEHICLE_LENGTH / 2
        half_w = VEHICLE_WIDTH / 2
        corners_local = np.array([
            [-half_l, -half_w],
            [ half_l, -half_w],
            [ half_l,  half_w],
            [-half_l,  half_w]
        ])
        rot_mat = np.array([[np.cos(yaw), -np.sin(yaw)],
                            [np.sin(yaw),  np.cos(yaw)]])
        corners_global = corners_local @ rot_mat.T + np.array([cx, cy])

        # 根据速度获取颜色
        rect_color = cmap(v_norm[i])

        polygon = plt.Polygon(corners_global, closed=True,
                              linewidth=2, edgecolor=rect_color,
                              facecolor='none', alpha=0.8)
        ax.add_patch(polygon)

    # 添加颜色条
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(v.min(), v.max()))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax)
    cbar.set_label('Velocity (m/s)')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Vehicle Trajectory with Velocity')
    ax.axis('equal')
    ax.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'xy_path.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"XY path plot saved to {out_dir}")

def plot_velocity(df, out_dir):
    """绘制速度-时间图"""
    fig, ax = plt.subplots(figsize=(8, 4))

    t = df['timestamp'].values - df['timestamp'].iloc[0]  # 相对时间
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

def plot_acceleration_scatter(df, out_dir):
    """绘制加速度散点图（ax vs ay）"""
    fig, ax = plt.subplots(figsize=(6, 6))

    ax_scatter = ax.scatter(df['ax'], df['ay'], c=df['timestamp'], cmap='viridis', s=10, alpha=0.6)
    ax.set_xlabel('ax (m/s²)')
    ax.set_ylabel('ay (m/s²)')
    ax.set_title('Acceleration Distribution')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.axhline(0, color='gray', linewidth=0.5)
    ax.axvline(0, color='gray', linewidth=0.5)

    cbar = plt.colorbar(ax_scatter, ax=ax)
    cbar.set_label('Time (s)')

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'acceleration_scatter.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Acceleration scatter plot saved to {out_dir}")

def plot_curvature(df, out_dir):
    """绘制曲率-时间图"""
    fig, ax = plt.subplots(figsize=(8, 4))

    t = df['timestamp'].values - df['timestamp'].iloc[0]
    kappa = df['kappa'].values

    ax.plot(t, kappa, 'g-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Curvature (1/m)')
    ax.set_title('Curvature Profile')
    ax.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'curvature.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Curvature plot saved to {out_dir}")

def main():
    parser = argparse.ArgumentParser(description='Plot test local planning results.')
    parser.add_argument('folder', nargs='?', help='Path to the folder containing vehicle_data.csv (optional, uses latest if not provided)')
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

    csv_path = os.path.join(folder, 'vehicle_data.csv')
    if not os.path.exists(csv_path):
        print(f"vehicle_data.csv not found in {folder}")
        sys.exit(1)

    # 读取数据
    df = pd.read_csv(csv_path)

    # 检查必要列
    required_cols = ['timestamp', 'x', 'y', 'v', 'ax', 'ay', 'kappa']
    for col in required_cols:
        if col not in df.columns:
            print(f"Missing column: {col}")
            sys.exit(1)

    # 调用各绘图函数
    plot_xy_with_velocity(df, folder)
    plot_velocity(df, folder)
    plot_acceleration_scatter(df, folder)
    plot_curvature(df, folder)

    # 显示合并图（2x2子图）
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))

    # XY 子图（简化版，仅轨迹线）
    ax1.plot(df['x'], df['y'], 'b-', linewidth=1)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Trajectory')
    ax1.axis('equal')
    ax1.grid(True, linestyle='--', alpha=0.7)

    # 速度子图
    t = df['timestamp'] - df['timestamp'].iloc[0]
    ax2.plot(t, df['v'], 'r-')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Velocity')
    ax2.grid(True, linestyle='--', alpha=0.7)

    # 加速度散点子图
    sc = ax3.scatter(df['ax'], df['ay'], c=t, cmap='viridis', s=5)
    ax3.set_xlabel('ax (m/s²)')
    ax3.set_ylabel('ay (m/s²)')
    ax3.set_title('Acceleration')
    ax3.grid(True, linestyle='--', alpha=0.7)
    plt.colorbar(sc, ax=ax3, label='Time (s)')

    # 曲率子图
    ax4.plot(t, df['kappa'], 'g-')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Curvature (1/m)')
    ax4.set_title('Curvature')
    ax4.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.savefig(os.path.join(folder, 'combined.png'), dpi=150, bbox_inches='tight')
    plt.show()
    print(f"Combined plot saved to {folder}")

if __name__ == "__main__":
    main()