#!/usr/bin/env python3
"""
Speed planning result visualization script.
Reads CSV file with columns: t,s,v,a,s_lower,s_upper,v_lower,v_upper,a_lower,a_upper
Displays ST diagram with obstacle polygons, velocity profile, and acceleration profile.
"""

import sys
import os
import glob
import argparse
from io import StringIO
import re

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


def find_latest_folder(base_dir):
    folders = glob.glob(os.path.join(base_dir, "????????_??????_*"))
    if not folders:
        return None
    return max(folders, key=os.path.getmtime)


def parse_obstacle_comments(comment_line):
    """解析障碍物注释行，返回障碍物字典列表。"""
    obstacles = []
    if not comment_line.startswith("# obstacles:"):
        return obstacles
    line = comment_line.replace("# obstacles:", "").strip()
    parts = []
    current = ""
    in_bracket = False
    for ch in line:
        if ch == '[':
            in_bracket = True
            current += ch
        elif ch == ']':
            in_bracket = False
            current += ch
            parts.append(current)
            current = ""
        elif ch == ' ' and not in_bracket:
            if current:
                parts.append(current)
                current = ""
        else:
            current += ch
    if current:
        parts.append(current)

    for part in parts:
        if part.startswith("static"):
            # 格式: static[20-25,margin=2.0]
            match = re.match(r'static\[([\d.]+)-([\d.]+),margin=([\d.]+)\]', part)
            if match:
                s_start, s_end, margin = map(float, match.groups())
                obstacles.append({
                    'type': 'static',
                    's_start': s_start,
                    's_end': s_end,
                    'margin': margin
                })
        elif part.startswith("dynamic"):
            # 格式: dynamic[s0=15.0,v=2.0,len=4.0,t=0.0-15.0,overtake=true,margin=2.0]
            match = re.match(
                r'dynamic\[s0=([\d.]+),v=([\d.]+),len=([\d.]+),t=([\d.]+)-([\d.]+),overtake=(\w+),margin=([\d.]+)\]',
                part
            )
            if match:
                s0, v, length, t_start, t_end, overtake_str, margin = match.groups()
                overtake = overtake_str.lower() == 'true'
                obstacles.append({
                    'type': 'dynamic',
                    's0': float(s0),
                    'v': float(v),
                    'length': float(length),
                    't_start': float(t_start),
                    't_end': float(t_end),
                    'overtake': overtake,
                    'margin': float(margin)
                })
    return obstacles


def cap_boundary(series, max_val, threshold=1e6, factor=1.2):
    """将超过阈值的边界值替换为 max_val * factor。"""
    capped = series.copy()
    mask = capped > threshold
    if mask.any():
        capped[mask] = max_val * factor
    return capped


def plot_st_obstacles(ax, obstacles, t_min, t_max):
    """在 ST 子图上绘制障碍物区域（实际区域 + 安全距离）。"""
    for obs in obstacles:
        if obs['type'] == 'static':
            margin = obs['margin']
            # 安全距离区域（底层）
            rect_margin = plt.Rectangle(
                (t_min, obs['s_start'] - margin),
                t_max - t_min,
                obs['s_end'] - obs['s_start'] + 2 * margin,
                alpha=0.2, color='red', zorder=4, label='safety margin'
            )
            ax.add_patch(rect_margin)
            # 实际障碍物区域（上层）
            rect = plt.Rectangle(
                (t_min, obs['s_start']),
                t_max - t_min,
                obs['s_end'] - obs['s_start'],
                alpha=0.7, color='red', zorder=5, label='obstacle'
            )
            ax.add_patch(rect)

        elif obs['type'] == 'dynamic':
            t0 = obs['t_start']
            t1 = obs['t_end']
            s0 = obs['s0']
            v = obs['v']
            length = obs['length']
            margin = obs['margin']

            # 实际障碍物多边形（不含 margin）
            s_front0 = s0 + length / 2
            s_rear0 = s0 - length / 2
            s_front1 = s0 + v * (t1 - t0) + length / 2
            s_rear1 = s0 + v * (t1 - t0) - length / 2
            polygon = Polygon([
                (t0, s_rear0),
                (t1, s_rear1),
                (t1, s_front1),
                (t0, s_front0)
            ], closed=True, alpha=0.7, color='orange', zorder=5, label='obstacle')
            ax.add_patch(polygon)

            # 安全距离多边形（含 margin）
            s_front0_m = s0 + length / 2 + margin
            s_rear0_m = s0 - length / 2 - margin
            s_front1_m = s0 + v * (t1 - t0) + length / 2 + margin
            s_rear1_m = s0 + v * (t1 - t0) - length / 2 - margin
            polygon_m = Polygon([
                (t0, s_rear0_m),
                (t1, s_rear1_m),
                (t1, s_front1_m),
                (t0, s_front0_m)
            ], closed=True, alpha=0.2, color='orange', zorder=4, label='safety margin')
            ax.add_patch(polygon_m)


def plot_csv(csv_path, out_dir):
    """读取 CSV 并绘图，保存图片到 out_dir。"""
    with open(csv_path, 'r') as f:
        lines = f.readlines()
    comment_lines = [l.strip() for l in lines if l.startswith('#')]
    data_lines = [l.strip() for l in lines if not l.startswith('#') and l.strip()]

    if len(data_lines) < 2:
        print("CSV file has no data lines.")
        return

    csv_data = '\n'.join(data_lines)
    data = pd.read_csv(StringIO(csv_data))

    obstacles = []
    for comment in comment_lines:
        obstacles.extend(parse_obstacle_comments(comment))

    t = data['t'].values
    s = data['s'].values
    v = data['v'].values
    a = data['a'].values
    s_lower = data['s_lower'].values
    s_upper = data['s_upper'].values
    v_lower = data['v_lower'].values
    v_upper = data['v_upper'].values
    a_lower = data['a_lower'].values
    a_upper = data['a_upper'].values

    max_s = np.max(s[s < 1e6]) if np.any(s < 1e6) else 1.0
    max_v = np.max(v[v < 1e6]) if np.any(v < 1e6) else 1.0
    max_a = np.max(np.abs(a[a < 1e6])) if np.any(a < 1e6) else 1.0

    s_upper_capped = cap_boundary(s_upper, max_s)
    v_upper_capped = cap_boundary(v_upper, max_v)
    a_upper_capped = cap_boundary(a_upper, max_a)
    a_lower_capped = cap_boundary(-a_lower, max_a)

    # ---------- 合并图 ----------
    plt.figure(figsize=(12, 10))

    # ST 子图
    ax1 = plt.subplot(3, 1, 1)
    ax1.plot(t, s, 'b-', label='s(t)', zorder=10)
    ax1.fill_between(t, s_lower, s_upper_capped, color='lightblue', alpha=0.3,
                     zorder=1, label='feasible region')
    plot_st_obstacles(ax1, obstacles, t[0], t[-1])
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Longitudinal position s (m)')
    ax1.set_title('ST Diagram with Obstacles')
    ax1.grid(True)
    # 图例放在左上角
    handles, labels = ax1.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax1.legend(by_label.values(), by_label.keys(), loc='upper left')

    # 速度子图
    ax2 = plt.subplot(3, 1, 2)
    ax2.plot(t, v, 'r-', label='v(t)', zorder=10)
    ax2.fill_between(t, v_lower, v_upper_capped, color='lightcoral', alpha=0.3,
                     zorder=1, label='speed limits')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('Velocity Profile')
    ax2.grid(True)
    ax2.legend(loc='upper right')

    # 加速度子图
    ax3 = plt.subplot(3, 1, 3)
    ax3.plot(t, a, 'g-', label='a(t)', zorder=10)
    ax3.fill_between(t, a_lower, a_upper_capped, color='lightgreen', alpha=0.3,
                     zorder=1, label='acceleration bounds')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Acceleration (m/s²)')
    ax3.set_title('Acceleration Profile')
    ax3.grid(True)
    ax3.legend(loc='upper right')

    plt.tight_layout()
    plt.show()

    # ---------- 保存独立子图 ----------
    # ST 图
    fig_st, ax_st = plt.subplots(figsize=(8, 6))
    ax_st.plot(t, s, 'b-', label='s(t)', zorder=10)
    ax_st.fill_between(t, s_lower, s_upper_capped, color='lightblue', alpha=0.3,
                       zorder=1, label='feasible region')
    plot_st_obstacles(ax_st, obstacles, t[0], t[-1])
    ax_st.set_xlabel('Time (s)')
    ax_st.set_ylabel('Longitudinal position s (m)')
    ax_st.set_title('ST Diagram with Obstacles')
    ax_st.grid(True)
    handles, labels = ax_st.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax_st.legend(by_label.values(), by_label.keys(), loc='upper left')
    fig_st.tight_layout()
    fig_st.savefig(os.path.join(out_dir, 'st_diagram.png'), dpi=150, bbox_inches='tight')
    plt.close(fig_st)

    # 速度图
    fig_v, ax_v = plt.subplots(figsize=(8, 6))
    ax_v.plot(t, v, 'r-', label='v(t)', zorder=10)
    ax_v.fill_between(t, v_lower, v_upper_capped, color='lightcoral', alpha=0.3,
                      zorder=1, label='speed limits')
    ax_v.set_xlabel('Time (s)')
    ax_v.set_ylabel('Velocity (m/s)')
    ax_v.set_title('Velocity Profile')
    ax_v.grid(True)
    ax_v.legend(loc='upper right')
    fig_v.tight_layout()
    fig_v.savefig(os.path.join(out_dir, 'velocity_profile.png'), dpi=150, bbox_inches='tight')
    plt.close(fig_v)

    # 加速度图
    fig_a, ax_a = plt.subplots(figsize=(8, 6))
    ax_a.plot(t, a, 'g-', label='a(t)', zorder=10)
    ax_a.fill_between(t, a_lower, a_upper_capped, color='lightgreen', alpha=0.3,
                      zorder=1, label='acceleration bounds')
    ax_a.set_xlabel('Time (s)')
    ax_a.set_ylabel('Acceleration (m/s²)')
    ax_a.set_title('Acceleration Profile')
    ax_a.grid(True)
    ax_a.legend(loc='upper right')
    fig_a.tight_layout()
    fig_a.savefig(os.path.join(out_dir, 'acceleration_profile.png'), dpi=150, bbox_inches='tight')
    plt.close(fig_a)

    print(f"Plots saved in: {out_dir}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot speed planning results.')
    parser.add_argument('csv_file', nargs='?',
                        help='Path to CSV file (optional, uses latest folder if not provided)')
    args = parser.parse_args()

    if args.csv_file:
        csv_path = args.csv_file
        out_dir = os.path.dirname(csv_path)
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base_dir = os.path.abspath(os.path.join(script_dir, '../result/test/speed_planning/'))
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