#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path planning result visualization script.
Reads CSV file with columns: s,x,y,kappa,obstacle_distance
Displays curvature vs. arc length and obstacle distance vs. arc length.
"""

import os
import sys
import glob
import argparse

import pandas as pd
import matplotlib.pyplot as plt


def find_latest_folder(base_dir):
    """返回 base_dir 下最新修改的文件夹（按修改时间）"""
    folders = glob.glob(os.path.join(base_dir, "????????_??????"))
    if not folders:
        return None
    return max(folders, key=os.path.getmtime)


def plot_csv(csv_path, out_dir):
    """读取 CSV 文件并绘制图像，保存到 out_dir。"""
    # 读取数据
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    # 检查必要的列
    required_cols = ['s', 'kappa', 'obstacle_distance']
    for col in required_cols:
        if col not in df.columns:
            print(f"CSV missing required column: {col}")
            return

    s = df['s'].values
    kappa = df['kappa'].values
    obs_dist = df['obstacle_distance'].values

    # ---------- 合并图（两个子图）----------
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # 曲率子图
    ax1.plot(s, kappa, 'b-', linewidth=2, label='Curvature')
    ax1.set_ylabel('Curvature [1/m]')
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.legend(loc='upper right')
    ax1.set_title('Path Curvature vs Arc Length')

    # 障碍物距离子图
    ax2.plot(s, obs_dist, 'r-', linewidth=2, label='Obstacle Distance')
    ax2.set_xlabel('Arc Length s [m]')
    ax2.set_ylabel('Distance [m]')
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.legend(loc='upper right')
    ax2.set_title('Distance to Nearest Obstacle vs Arc Length')

    plt.tight_layout()
    plt.show()

    # ---------- 保存独立子图 ----------
    # 曲率图
    fig_kappa, ax_kappa = plt.subplots(figsize=(8, 6))
    ax_kappa.plot(s, kappa, 'b-', linewidth=2)
    ax_kappa.set_xlabel('Arc Length s [m]')
    ax_kappa.set_ylabel('Curvature [1/m]')
    ax_kappa.set_title('Path Curvature vs Arc Length')
    ax_kappa.grid(True, linestyle='--', alpha=0.7)
    fig_kappa.tight_layout()
    fig_kappa.savefig(os.path.join(out_dir, 'curvature.png'), dpi=150, bbox_inches='tight')
    plt.close(fig_kappa)

    # 障碍物距离图
    fig_dist, ax_dist = plt.subplots(figsize=(8, 6))
    ax_dist.plot(s, obs_dist, 'r-', linewidth=2)
    ax_dist.set_xlabel('Arc Length s [m]')
    ax_dist.set_ylabel('Distance [m]')
    ax_dist.set_title('Distance to Nearest Obstacle vs Arc Length')
    ax_dist.grid(True, linestyle='--', alpha=0.7)
    fig_dist.tight_layout()
    fig_dist.savefig(os.path.join(out_dir, 'obstacle_distance.png'), dpi=150, bbox_inches='tight')
    plt.close(fig_dist)

    print(f"Plots saved in: {out_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot path planning results.')
    parser.add_argument('csv_file', nargs='?',
                        help='Path to CSV file (optional, uses latest folder if not provided)')
    args = parser.parse_args()

    if args.csv_file:
        csv_path = args.csv_file
        out_dir = os.path.dirname(csv_path)
    else:
        # 自动查找最新生成的数据文件夹
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base_dir = os.path.abspath(os.path.join(script_dir, '../result/test/path_planning/'))
        if not os.path.exists(base_dir):
            print(f"Base directory not found: {base_dir}")
            sys.exit(1)

        latest_folder = find_latest_folder(base_dir)
        if latest_folder is None:
            print("No experiment folders found in", base_dir)
            sys.exit(1)

        csv_path = os.path.join(latest_folder, 'data.csv')
        out_dir = latest_folder
        print(f"Using latest folder: {latest_folder}")

    if not os.path.exists(csv_path):
        print(f"CSV file not found: {csv_path}")
        sys.exit(1)

    plot_csv(csv_path, out_dir)