#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test Local Planning Data Visualization (Dynamic)
Reads vehicle_data.csv and planning_times.csv, also loads obstacle config from YAML.
Plots:
1. XY trajectory with filled rectangles
2. Velocity vs time
3. Ax vs Ay scatter
4. Ax vs time
5. Curvature vs time
6. Planning time vs index
7. Terrain distance vs time
8. Obstacle distance vs time
"""

import os
import sys
import glob
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
from mpl_toolkits.axes_grid1 import make_axes_locatable
import yaml


# ==================== 硬编码障碍物配置文件路径 ====================
OBSTACLE_CONFIG_PATH = "/home/brucesun/GraduationProject/src/perception/config/dynamic_experiment/XG.yaml"  # 请修改为实际路径
# OBSTACLE_CONFIG_PATH = "/home/brucesun/GraduationProject/src/perception/config/dynamic_experiment/carla.yaml"  # 请修改为实际路径
# ==================== 参数  ====================
VEHICLE_LENGTH = 3.8
VEHICLE_WIDTH = 2.0
DOWNSAMPLE_STEP = 10


# ==================== 障碍物数据加载 ====================
def load_obstacle_config(yaml_path):
    """加载 YAML 障碍物配置文件"""
    if not os.path.exists(yaml_path):
        print(f"Warning: Obstacle config file {yaml_path} not found. Obstacles will not be plotted.")
        return []
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    obstacles = []
    for obs in config.get('obstacles', []):
        obstacles.append({
            'id': obs['id'],
            'name': obs.get('name', ''),
            'init_x': obs['init_x'],
            'init_y': obs['init_y'],
            'init_yaw': obs['init_yaw'],
            'length': obs['length'],
            'width': obs['width'],
            'speed': obs['speed'],
        })
    return obstacles


# 全局障碍物列表
OBSTACLES = load_obstacle_config(OBSTACLE_CONFIG_PATH)


# ==================== 辅助函数 ====================
def find_latest_folder(base_dir):
    folders = glob.glob(os.path.join(base_dir, "????????_??????"))
    if not folders:
        return None
    return max(folders, key=os.path.getmtime)


def create_vehicle_polygon(x, y, yaw, length, width):
    """创建表示车辆的矩形多边形（中心点 + 朝向）"""
    half_l = length / 2.0
    half_w = width / 2.0
    corners_local = np.array([
        [-half_l, -half_w],
        [ half_l, -half_w],
        [ half_l,  half_w],
        [-half_l,  half_w]
    ])
    rot_mat = np.array([[np.cos(yaw), -np.sin(yaw)],
                        [np.sin(yaw),  np.cos(yaw)]])
    corners_global = corners_local @ rot_mat.T + np.array([x, y])
    return Polygon(corners_global, closed=True)


def plot_xy_with_velocity(df, out_dir):
    """绘制 XY 轨迹，用实心矩形表示车辆，颜色代表速度"""
    # 提取自车数据
    ego_x = df['x'].values
    ego_y = df['y'].values
    ego_theta = df['theta'].values
    ego_v = df['v'].values

    # 收集所有速度（自车 + 障碍物）用于统一颜色映射
    all_v = list(ego_v)
    for obs in OBSTACLES:
        # 障碍物速度恒定，但为了颜色映射，每个时间点都相同
        all_v.extend([obs['speed']] * len(ego_x))
    v_min = min(all_v)
    v_max = max(all_v)
    norm = plt.Normalize(v_min, v_max)
    cmap = cm.jet

    # 计算合适的图形尺寸
    x_range = ego_x.max() - ego_x.min()
    y_range = ego_y.max() - ego_y.min()
    fig_width = 12
    fig_height = 8
    fig, ax = plt.subplots(figsize=(fig_width, fig_height))

    # 绘制自车矩形
    ego_patches = []
    ego_colors = []
    for i in range(len(ego_x)):
        poly = create_vehicle_polygon(ego_x[i], ego_y[i], ego_theta[i],
                                      VEHICLE_LENGTH, VEHICLE_WIDTH)
        ego_patches.append(poly)
        ego_colors.append(ego_v[i])
    # 一次性添加所有自车矩形以提高性能
    ego_collection = PatchCollection(ego_patches, cmap=cmap, norm=norm, edgecolor='black', linewidth=1)
    ego_collection.set_array(np.array(ego_colors))
    ego_collection.set_alpha(0.8)
    ax.add_collection(ego_collection)

    # 绘制障碍物矩形
    if OBSTACLES:
        t_rel = df['timestamp'].values - df['timestamp'].iloc[0]  # 相对时间
        for obs in OBSTACLES:
            # 计算障碍物在每个时间点的位置（匀速直线运动，航向不变）
            obs_x = obs['init_x'] + obs['speed'] * np.cos(obs['init_yaw']) * t_rel
            obs_y = obs['init_y'] + obs['speed'] * np.sin(obs['init_yaw']) * t_rel
            obs_yaw = np.full_like(t_rel, obs['init_yaw'])  # 航向不变
            obs_v = np.full_like(t_rel, obs['speed'])

            obs_patches = []
            for i in range(len(obs_x)):
                poly = create_vehicle_polygon(obs_x[i], obs_y[i], obs_yaw[i],
                                              obs['length'], obs['width'])
                obs_patches.append(poly)
            obs_collection = PatchCollection(obs_patches, cmap=cmap, norm=norm,
                                             edgecolor='gray', linewidth=1)
            obs_collection.set_array(obs_v)
            obs_collection.set_alpha(0.8)
            ax.add_collection(obs_collection)

    # 参考线（起点到终点的浅绿色虚线）
    start_x, start_y = ego_x[0], ego_y[0]
    end_x, end_y = ego_x[-1], ego_y[-1]
    ax.plot([start_x, end_x], [start_y, end_y], 'c--', linewidth=2, alpha=0.5, label='Reference line')

    ax.autoscale()
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    # ax.set_xlim(40, 110)
    # ax.set_ylim(330, 370)
    ax.set_title('Vehicle Trajectory with Velocity')
    ax.grid(True, linestyle='--', alpha=0.7)

    # 自定义图例（空心矩形）
    from matplotlib.patches import Rectangle
    legend_elements = [
        Rectangle((0, 0), 1, 1, linewidth=2, edgecolor='black', facecolor='none', label='Ego vehicle'),
        Rectangle((0, 0), 1, 1, linewidth=2, edgecolor='gray', facecolor='none', label='Obstacle'),
        plt.Line2D([0], [0], color='cyan', linestyle='--', linewidth=2, label='Reference line')
    ]
    ax.legend(handles=legend_elements, loc='upper right')

    # 颜色条
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, cax=cax)
    cbar.set_label('Velocity (m/s)')

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'xy_path.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"XY path plot saved to {out_dir}")


def plot_velocity(df, out_dir):
    fig, ax = plt.subplots(figsize=(8, 4))
    t = df['timestamp'].values - df['timestamp'].iloc[0]
    ax.plot(t, df['v'], 'b-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Velocity Profile')
    ax.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'velocity.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Velocity plot saved to {out_dir}")


def plot_ax_vs_ay(df, out_dir):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(df['ay'], df['ax'], c='b', s=40, alpha=0.6, edgecolors='none')
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
    fig, ax = plt.subplots(figsize=(8, 4))
    t = df['timestamp'].values - df['timestamp'].iloc[0]
    ax.plot(t, df['ax'], 'm-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('ax (m/s²)')
    ax.set_title('Longitudinal Acceleration Profile')
    ax.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'ax_t.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Ax-t plot saved to {out_dir}")


def plot_curvature(df, out_dir):
    fig, ax = plt.subplots(figsize=(8, 4))
    t = df['timestamp'].values - df['timestamp'].iloc[0]
    ax.plot(t, df['kappa'], 'g-', linewidth=2)
    max_abs = np.abs(df['kappa']).max() * 1.05
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
    fig, ax = plt.subplots(figsize=(8, 4))
    idx = df_times['index'].values
    cost = df_times['cost_time_ms'].values
    ax.plot(idx, cost, 'c-', linewidth=2, label='Planning time')
    # ax.set_ylim(0, 100)
    ax.set_ylim(0, 60)
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
    y_max = dist.max()
    ax.set_ylim(bottom=0, top=y_max * 1.05 if y_max > 0 else 1.0)
    min_val = dist.min()
    ax.axhline(y=min_val, color='gray', linestyle='--', linewidth=2,
               label=f'Min: {min_val:.2f} m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Terrain distance (m)')
    ax.set_title('Terrain Distance Profile')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.legend(loc='upper right')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'terrain_distance.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Terrain distance plot saved to {out_dir}")


def plot_obstacle_distance(df, out_dir):
    fig, ax = plt.subplots(figsize=(8, 4))
    t = df['timestamp'].values - df['timestamp'].iloc[0]
    dist = df['obstacle_distance'].values
    valid = dist >= 0
    if np.any(valid):
        ax.plot(t[valid], dist[valid], color='purple', linewidth=2, label='Obstacle distance')
        y_max = dist[valid].max()
        ax.set_ylim(bottom=0, top=y_max * 1.05 if y_max > 0 else 1.0)
        min_val = dist[valid].min()
        ax.axhline(y=min_val, color='gray', linestyle='--', linewidth=2,
                   label=f'Min: {min_val:.2f} m')
    else:
        ax.text(0.5, 0.5, 'No obstacle data', transform=ax.transAxes, ha='center')
        ax.set_ylim(0, 1)  # 默认范围
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Obstacle distance (m)')
    ax.set_title('Obstacle Distance Profile')
    ax.grid(True, linestyle='--', alpha=0.7)
    if np.any(valid):
        ax.legend(loc='upper right')
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, 'obstacle_distance.png'), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Obstacle distance plot saved to {out_dir}")


def main():
    parser = argparse.ArgumentParser(description='Plot test local planning results (dynamic).')
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
    df_vehicle = pd.read_csv(vehicle_csv)
    required_cols = ['timestamp', 'x', 'y', 'theta', 'v', 'ax', 'ay', 'kappa', 'terrain_distance', 'obstacle_distance']
    for col in required_cols:
        if col not in df_vehicle.columns:
            print(f"Missing column: {col} in vehicle_data.csv")
            sys.exit(1)

    # 读取规划耗时数据
    times_csv = os.path.join(folder, 'planning_times.csv')
    if not os.path.exists(times_csv):
        print(f"planning_times.csv not found in {folder}")
        sys.exit(1)
    df_times = pd.read_csv(times_csv)
    required_time_cols = ['index', 'cost_time_ms']
    for col in required_time_cols:
        if col not in df_times.columns:
            print(f"Missing column: {col} in planning_times.csv")
            sys.exit(1)

    # 保存独立图片
    plot_xy_with_velocity(df_vehicle.iloc[::DOWNSAMPLE_STEP], folder)
    plot_velocity(df_vehicle, folder)
    plot_ax_vs_ay(df_vehicle, folder)
    plot_ax_t(df_vehicle, folder)
    plot_curvature(df_vehicle, folder)
    plot_planning_times(df_times, folder)
    plot_terrain_distance(df_vehicle, folder)
    plot_obstacle_distance(df_vehicle, folder)

    # 组合图（3行3列，顺序：1,2,4,3,5,6,7,8）
    fig, axes = plt.subplots(3, 3, figsize=(18, 18))

    # 图1: XY轨迹（放在 (0,0)）
    ax1 = axes[0, 0]
    ego_x = df_vehicle['x'].values
    ego_y = df_vehicle['y'].values
    ego_theta = df_vehicle['theta'].values
    ego_v = df_vehicle['v'].values

    # 收集所有速度用于颜色映射
    all_v = list(ego_v)
    for obs in OBSTACLES:
        all_v.extend([obs['speed']] * len(ego_x))
    v_min = min(all_v)
    v_max = max(all_v)
    norm = plt.Normalize(v_min, v_max)
    cmap = cm.jet

    # 自车矩形
    ego_patches = []
    for i in range(len(ego_x)):
        poly = create_vehicle_polygon(ego_x[i], ego_y[i], ego_theta[i],
                                      VEHICLE_LENGTH, VEHICLE_WIDTH)
        ego_patches.append(poly)
    ego_collection = PatchCollection(ego_patches, cmap=cmap, norm=norm, edgecolor='black', linewidth=1)
    ego_collection.set_array(ego_v)
    ego_collection.set_alpha(0.8)
    ax1.add_collection(ego_collection)

    # 障碍物矩形
    if OBSTACLES:
        t_rel = df_vehicle['timestamp'].values - df_vehicle['timestamp'].iloc[0]
        for obs in OBSTACLES:
            obs_x = obs['init_x'] + obs['speed'] * np.cos(obs['init_yaw']) * t_rel
            obs_y = obs['init_y'] + obs['speed'] * np.sin(obs['init_yaw']) * t_rel
            obs_yaw = np.full_like(t_rel, obs['init_yaw'])
            obs_v = np.full_like(t_rel, obs['speed'])

            obs_patches = []
            for i in range(len(obs_x)):
                poly = create_vehicle_polygon(obs_x[i], obs_y[i], obs_yaw[i],
                                              obs['length'], obs['width'])
                obs_patches.append(poly)
            obs_collection = PatchCollection(obs_patches, cmap=cmap, norm=norm,
                                             edgecolor='gray', linewidth=1)
            obs_collection.set_array(obs_v)
            obs_collection.set_alpha(0.8)
            ax1.add_collection(obs_collection)

    # 参考线
    ax1.plot([ego_x[0], ego_x[-1]], [ego_y[0], ego_y[-1]], 'c--', linewidth=2, alpha=0.5, label='Reference line')
    ax1.autoscale()
    ax1.set_aspect('equal')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    # ax1.set_xlim(40, 110)
    # ax1.set_ylim(330, 370)
    ax1.set_title('Trajectory with Velocity')
    ax1.grid(True, linestyle='--', alpha=0.7)

    # 图例
    from matplotlib.patches import Rectangle
    legend_elements = [
        Rectangle((0, 0), 1, 1, linewidth=2, edgecolor='black', facecolor='none', label='Ego vehicle'),
        Rectangle((0, 0), 1, 1, linewidth=2, edgecolor='gray', facecolor='none', label='Obstacle'),
        plt.Line2D([0], [0], color='cyan', linestyle='--', linewidth=2, label='Reference line')
    ]
    ax1.legend(handles=legend_elements, loc='upper right')

    # 颜色条
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax1, fraction=0.05, pad=0.04)
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

    # 图7: terrain distance
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

    # 图8: obstacle distance
    ax8 = axes[2, 1]
    obs_dist = df_vehicle['obstacle_distance'].values
    valid = obs_dist >= 0
    if np.any(valid):
        ax8.plot(t[valid], obs_dist[valid], color='purple', linewidth=2, label='Obstacle distance')
        y_max = obs_dist[valid].max()
        ax8.set_ylim(bottom=0, top=y_max * 1.05 if y_max > 0 else 1.0)
        min_val = obs_dist[valid].min()
        ax8.axhline(y=min_val, color='gray', linestyle='--', linewidth=2,
                    label=f'Min: {min_val:.2f} m')
    else:
        ax8.text(0.5, 0.5, 'No obstacle data', transform=ax8.transAxes, ha='center')
        ax8.set_ylim(0, 1)
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Obstacle distance (m)')
    ax8.set_title('Obstacle Distance')
    ax8.grid(True, linestyle='--', alpha=0.7)
    if np.any(valid):
        ax8.legend(loc='upper right')

    # 最后一列 (2,2) 留空
    axes[2, 2].axis('off')

    plt.tight_layout()
    plt.savefig(os.path.join(folder, 'combined.png'), dpi=150, bbox_inches='tight')
    plt.show()
    print(f"Combined plot saved to {folder}")


if __name__ == "__main__":
    main()