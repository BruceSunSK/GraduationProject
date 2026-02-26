#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Decision maker result visualization.
Reads path_boundary.csv and speed_boundary.csv from the latest folder,
plots SL diagram with obstacles and ST diagram with obstacles.
"""

import os
import sys
import glob
import argparse
from io import StringIO
import re

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon


def find_latest_folder(base_dir):
    folders = glob.glob(os.path.join(base_dir, "????????_??????"))
    if not folders:
        return None
    return max(folders, key=os.path.getmtime)


def parse_obstacle_comments(lines, prefix):
    """从注释行解析障碍物信息。返回障碍物列表，每个为字典。"""
    obstacles = []
    for line in lines:
        if not line.startswith(prefix):
            continue
        # 格式示例: # obstacle: id=1, s=10.5, l=1.2, length=4.0, width=2.0, speed=2.0
        # 使用正则提取
        pattern = r'id=(\d+),\s*s=([\d.-]+),\s*l=([\d.-]+),\s*length=([\d.-]+),\s*width=([\d.-]+),\s*speed=([\d.-]+)'
        match = re.search(pattern, line)
        if match:
            obs = {
                'id': int(match.group(1)),
                's': float(match.group(2)),
                'l': float(match.group(3)),
                'length': float(match.group(4)),
                'width': float(match.group(5)),
                'speed': float(match.group(6))
            }
            obstacles.append(obs)
    return obstacles


def plot_sl_diagram(csv_path, out_dir):
    with open(csv_path, 'r') as f:
        lines = f.readlines()
    comment_lines = [l.strip() for l in lines if l.startswith('#')]
    data_lines = [l.strip() for l in lines if not l.startswith('#') and l.strip()]

    if len(data_lines) < 2:
        print("No data in path_boundary.csv")
        return

    csv_data = '\n'.join(data_lines)
    df = pd.read_csv(StringIO(csv_data))

    obstacles = parse_obstacle_comments(comment_lines, '# obstacle')

    s = df['s'].values
    l_lower = df['l_lower'].values
    l_upper = df['l_upper'].values

    fig, ax = plt.subplots(figsize=(10, 6))

    # 填充可行区域
    ax.fill_between(s, l_lower, l_upper, color='lightblue', alpha=0.5, label='feasible region')

    # 绘制障碍物矩形
    for obs in obstacles:
        rect = Rectangle(
            (obs['s'] - obs['length']/2, obs['l'] - obs['width']/2),
            obs['length'], obs['width'],
            linewidth=2, edgecolor='red', facecolor='red', alpha=0.7, label='obstacle' if obs['id'] == obstacles[0]['id'] else ""
        )
        ax.add_patch(rect)

    # 绘制自车轮廓，位于规划起点（第一个 s 值处）
    ego_length = 3.4
    ego_width = 2.0
    start_s = s[0]  # 规划起点对应的 s 坐标
    ego_rect = Rectangle(
        (start_s - ego_length/2, -ego_width/2),
        ego_length, ego_width,
        linewidth=2, edgecolor='gray', facecolor='lightgray', alpha=0.7, label='ego vehicle'
    )
    ax.add_patch(ego_rect)

    # 设置坐标轴标签
    ax.set_xlabel('s (m)')
    ax.set_ylabel('l (m)')
    ax.set_title('SL Diagram with Obstacles')
    ax.grid(True, linestyle='--', alpha=0.7)

    # 使纵轴关于0对称
    l_min = min(l_lower)
    l_max = max(l_upper)
    max_abs = max(abs(l_min), abs(l_max))
    ax.set_ylim(-max_abs, max_abs)

    ax.legend(loc='upper right')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'sl_diagram.png'), dpi=150, bbox_inches='tight')
    plt.show()
    print(f"SL diagram saved to {out_dir}")


def plot_st_diagram(csv_path, out_dir):
    with open(csv_path, 'r') as f:
        lines = f.readlines()
    comment_lines = [l.strip() for l in lines if l.startswith('#')]
    data_lines = [l.strip() for l in lines if not l.startswith('#') and l.strip()]

    if len(data_lines) < 2:
        print("No data in speed_boundary.csv")
        return

    csv_data = '\n'.join(data_lines)
    df = pd.read_csv(StringIO(csv_data))

    obstacles = parse_obstacle_comments(comment_lines, '# obstacle')

    t = df['t'].values
    s_lower = df['s_lower'].values
    s_upper = df['s_upper'].values

    fig, ax = plt.subplots(figsize=(10, 6))

    # 填充可行区域
    ax.fill_between(t, s_lower, s_upper, color='lightgreen', alpha=0.5, label='feasible region')

    # 绘制障碍物在 ST 图上的投影
    t_max = t[-1]
    for obs in obstacles:
        s = obs['s']
        v = obs['speed']
        length = obs['length']
        # 静态障碍物（速度接近0）用矩形表示
        if abs(v) < 0.1:
            rect = Rectangle(
                (0, s - length/2),
                t_max, length,
                linewidth=2, edgecolor='red', facecolor='red', alpha=0.7,
                label='static obstacle' if obs['id'] == obstacles[0]['id'] else ""
            )
            ax.add_patch(rect)
        else:  # 动态障碍物，用平行四边形表示（匀速假设）
            # 四个顶点：(0, s0 - L/2), (t_max, s0 + v*t_max - L/2),
            #          (t_max, s0 + v*t_max + L/2), (0, s0 + L/2)
            pts = np.array([
                [0, s - length/2],
                [t_max, s + v*t_max - length/2],
                [t_max, s + v*t_max + length/2],
                [0, s + length/2]
            ])
            poly = Polygon(pts, closed=True, edgecolor='red', facecolor='red', alpha=0.7,
                           label='dynamic obstacle' if obs['id'] == obstacles[0]['id'] else "")
            ax.add_patch(poly)

    ax.set_xlabel('t (s)')
    ax.set_ylabel('s (m)')
    ax.set_title('ST Diagram with Obstacles')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.legend(loc='upper right')

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'st_diagram.png'), dpi=150, bbox_inches='tight')
    plt.show()
    print(f"ST diagram saved to {out_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot decision maker results.')
    parser.add_argument('folder', nargs='?', help='Path to the folder containing CSV files (optional, uses latest if not provided)')
    args = parser.parse_args()

    if args.folder:
        folder = args.folder
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base_dir = os.path.abspath(os.path.join(script_dir, '../result/test/decision_maker/'))
        if not os.path.exists(base_dir):
            print(f"Base directory not found: {base_dir}")
            sys.exit(1)
        folder = find_latest_folder(base_dir)
        if folder is None:
            print("No data folders found in", base_dir)
            sys.exit(1)
        print(f"Using latest folder: {folder}")

    path_csv = os.path.join(folder, 'path_boundary.csv')
    speed_csv = os.path.join(folder, 'speed_boundary.csv')

    if os.path.exists(path_csv):
        plot_sl_diagram(path_csv, folder)
    else:
        print(f"path_boundary.csv not found in {folder}")

    if os.path.exists(speed_csv):
        plot_st_diagram(speed_csv, folder)
    else:
        print(f"speed_boundary.csv not found in {folder}")