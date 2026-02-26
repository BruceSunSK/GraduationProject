#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Manual ST Diagram Plotter
Allows custom specification of obstacles with time interval (t_start, t_end) and motion parameters.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

def plot_st_diagram_manual(obstacles=None, t_range=(0, 10), s_range=(0, 50)):
    """
    手动绘制 ST 图

    :param obstacles: 障碍物列表，每个元素为字典，包含：
                      - t_start: 障碍物开始时间 (s)
                      - t_end:   障碍物结束时间 (s)
                      - s0:      在 t_start 时刻的中心纵向位置 (m)
                      - v:       速度 (m/s)（匀速）
                      - length:  障碍物长度 (m)
                      - decision: 'overtake' 或 'yield'
                      （可选 'id'，用于调试）
    :param t_range: 整体时间范围 (t_min, t_max)
    :param s_range: 纵向位置范围 (s_min, s_max)
    """
    t_min, t_max = t_range
    s_min, s_max = s_range

    # 生成高分辨率时间点用于绘图和边界计算
    t = np.linspace(t_min, t_max, 1000)

    # 初始化可行域边界为整个矩形
    s_lower = np.full_like(t, s_min)
    s_upper = np.full_like(t, s_max)

    # 用于存储障碍物多边形
    obstacle_patches = []
    legend_handles = []

    # 处理每个障碍物
    for i, obs in enumerate(obstacles or []):
        t_start = obs['t_start']
        t_end   = obs['t_end']
        s0      = obs['s0']
        v       = obs['v']
        length  = obs['length']
        decision = obs.get('decision', 'yield').lower()
        obs_id = obs.get('id', i)

        # 检查时间区间有效性
        if t_start >= t_end:
            print(f"Warning: obstacle {obs_id} has t_start >= t_end, ignoring.")
            continue

        # 对于每个时间点，判断是否在障碍物存在区间内
        mask = (t >= t_start) & (t <= t_end)
        if not np.any(mask):
            # 障碍物完全不在绘图时间范围内，跳过
            continue

        # 计算障碍物中心位置随时间变化（仅在有效区间内）
        # 注意：障碍物在 t_start 时刻中心为 s0，之后匀速运动
        s_center = s0 + v * (t - t_start)
        lower_bound = s_center - length / 2
        upper_bound = s_center + length / 2

        # 仅在掩码区域内修改可行域边界
        if decision == 'overtake':
            # 可行域必须在障碍物上方：s >= upper_bound
            s_lower[mask] = np.maximum(s_lower[mask], upper_bound[mask])
        elif decision == 'yield':
            # 可行域必须在障碍物下方：s <= lower_bound
            s_upper[mask] = np.minimum(s_upper[mask], lower_bound[mask])
        else:
            print(f"Warning: Unknown decision '{decision}' for obstacle {obs_id}, ignoring.")
            continue

        # 计算障碍物平行四边形的四个顶点
        # 时刻 t_start 的中心 = s0
        # 时刻 t_end   的中心 = s0 + v * (t_end - t_start)
        s_center_end = s0 + v * (t_end - t_start)
        pts = np.array([
            [t_start, s0 - length/2],
            [t_end,   s_center_end - length/2],
            [t_end,   s_center_end + length/2],
            [t_start, s0 + length/2]
        ])
        poly = Polygon(pts, closed=True, edgecolor='red', facecolor='red', alpha=0.7,
                       label=f"{decision} obstacle" if i == 0 else "")
        obstacle_patches.append(poly)
        if i == 0:
            legend_handles.append(poly)

    # 确保边界合理（s_lower <= s_upper）
    s_lower = np.clip(s_lower, s_min, s_max)
    s_upper = np.clip(s_upper, s_lower, s_max)

    # 绘图
    fig, ax = plt.subplots(figsize=(10, 6))

    # 填充可行域
    ax.fill_between(t, s_lower, s_upper, color='lightgreen', alpha=0.5, label='feasible region')
    legend_handles.insert(0, plt.Rectangle((0,0), 1, 1, color='lightgreen', alpha=0.5, label='feasible region'))

    # 添加障碍物多边形
    for poly in obstacle_patches:
        ax.add_patch(poly)

    # ax.set_xlim(t_min, t_max)
    # ax.set_ylim(s_min, s_max)
    ax.set_xlabel('t (s)')
    ax.set_ylabel('s (m)')
    ax.set_title('ST Diagram with Obstacles')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.legend(handles=legend_handles, loc='upper right')

    plt.tight_layout()
    plt.savefig('st_diagram.png', dpi=150, bbox_inches='tight')
    plt.show()


if __name__ == "__main__":
    # 手动配置障碍物
    obstacles = [
        # {
        #     # cross, overtake
        #     't_start': 2.82,
        #     't_end':   4.55,
        #     's0':      22.13,
        #     'v':       2.2,
        #     'length':  3.2,
        #     'decision': 'overtake',
        #     'id': 1
        # },
        {
            # cross, yield
            't_start': 1.36,
            't_end':   2.40,
            's0':      25.13,
            'v':       5.0,
            'length':  3.2,
            'decision': 'yield',
            'id': 1
        },
    ]
    plot_st_diagram_manual(obstacles, t_range=(0, 5), s_range=(13.39, 38))