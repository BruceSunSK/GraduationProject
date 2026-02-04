#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import deque
import threading
import time
import os

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped

class Plotter:
    def __init__(self):
        rospy.init_node('pid_plotter', anonymous=True)
        
        # 参数配置
        self.max_data_points = 1000
        self.plot_update_rate = 10
        self.save_interval = 5.0
        
        # 获取control包路径
        import rospkg
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('control')
        
        # 创建保存目录
        self.plot_dir = os.path.join(package_path, 'plots')
        if not os.path.exists(self.plot_dir):
            os.makedirs(self.plot_dir)
        rospy.loginfo(f"[Plotter]: Plots will be saved to: {self.plot_dir}")
        
        # 初始化数据队列
        self.init_data_queues()
        
        # 初始化Matplotlib图形
        self.init_plots()
        
        # 使用线程安全锁
        self.lock = threading.Lock()

        # 跟踪数据接收状态和初始时间戳
        self.data_received = {
            'ref_traj': False,
            'act_traj': False,
            'odom': False
        }
        
        # 每个话题的初始时间戳
        self.initial_timestamps = {
            'ref_traj': None,
            'act_traj': None,
            'odom': None
        }
        
        # 上一次保存时间
        self.last_save_time = time.time()

        # 订阅话题
        rospy.Subscriber('/reference_trajectory', Path, self.reference_trajectory_callback)
        rospy.Subscriber('/actual_trajectory', Path, self.actual_trajectory_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        rospy.loginfo("[Plotter]: PID Plotter node started")
        
    def init_data_queues(self):
        """初始化数据存储队列"""
        # 参考轨迹数据
        self.ref_traj_times = deque(maxlen=self.max_data_points)
        self.ref_path_x = deque(maxlen=self.max_data_points)
        self.ref_path_y = deque(maxlen=self.max_data_points)
        self.ref_vel_x_data = deque(maxlen=self.max_data_points)
        
        # 实际轨迹数据
        self.act_traj_times = deque(maxlen=self.max_data_points)
        self.act_path_x = deque(maxlen=self.max_data_points)
        self.act_path_y = deque(maxlen=self.max_data_points)
        
        # 里程计数据
        self.odom_times = deque(maxlen=self.max_data_points)
        self.act_vel_x_data = deque(maxlen=self.max_data_points)
        self.act_vel_z_data = deque(maxlen=self.max_data_points)
        
    def init_plots(self):
        """初始化Matplotlib图形"""
        # 创建图形，1行3列
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(1, 3, figsize=(18, 6))
        
        # 设置子图样式
        self.setup_plot_style()
        
        # 初始化绘图线条
        self.init_plot_lines()
        
        plt.tight_layout()
        
    def setup_plot_style(self):
        """设置绘图样式"""
        # 轨迹图
        self.ax1.set_title('Reference vs Actual Trajectory')
        self.ax1.set_xlabel('X [m]')
        self.ax1.set_ylabel('Y [m]')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.axis('equal')
        
        # 线速度图
        self.ax2.set_title('Linear Velocity Tracking')
        self.ax2.set_xlabel('Time [s]')
        self.ax2.set_ylabel('Velocity [m/s]')
        self.ax2.grid(True, alpha=0.3)
        
        # 角速度图
        self.ax3.set_title('Angular Velocity')
        self.ax3.set_xlabel('Time [s]')
        self.ax3.set_ylabel('Angular Velocity [rad/s]')
        self.ax3.grid(True, alpha=0.3)
        
    def init_plot_lines(self):
        """初始化绘图线条"""
        # 轨迹图
        self.ref_path_line, = self.ax1.plot([], [], 'b-', label='Reference', linewidth=2, alpha=0.8)
        self.act_path_line, = self.ax1.plot([], [], 'r-', label='Actual', linewidth=1.5, alpha=0.8)
        self.ax1.legend(loc='upper right')
        
        # 线速度图
        self.ref_vel_x_line, = self.ax2.plot([], [], 'b-', label='Reference', linewidth=2, alpha=0.8)
        self.act_vel_x_line, = self.ax2.plot([], [], 'r-', label='Actual', linewidth=1.5, alpha=0.8)
        self.ax2.legend(loc='upper right')
        
        # 角速度图
        self.act_vel_z_line, = self.ax3.plot([], [], 'b-', label='Actual', linewidth=1.5, alpha=0.8)
        self.ax3.legend(loc='upper right')
        
    def get_relative_time(self, msg, topic_type):
        """
        获取相对于该话题第一次消息的时间
        topic_type: 'ref_traj', 'act_traj', 'odom'
        """
        # 获取消息时间戳
        if hasattr(msg, 'header'):
            msg_time = msg.header.stamp.to_sec()
        else:
            msg_time = rospy.Time.now().to_sec()
        
        # 如果是该话题的第一次消息，记录初始时间戳
        if self.initial_timestamps[topic_type] is None:
            self.initial_timestamps[topic_type] = msg_time
            rospy.loginfo(f"[Plotter]: First {topic_type} message at time {msg_time}")
            return 0.0
        
        # 计算相对于初始时间的时间
        return msg_time - self.initial_timestamps[topic_type]
        
    def reference_trajectory_callback(self, msg):
        """处理参考轨迹数据"""
        with self.lock:
            self.data_received['ref_traj'] = True
            
            # 清除旧数据
            self.ref_traj_times.clear()
            self.ref_path_x.clear()
            self.ref_path_y.clear()
            self.ref_vel_x_data.clear()
            
            # 计算相对于第一次参考轨迹消息的时间
            base_time = self.get_relative_time(msg, 'ref_traj')
            
            # 为每个参考点分配时间戳
            time_interval = 0.1  # 100ms间隔
            
            for i, pose in enumerate(msg.poses):
                x = pose.pose.position.x
                y = pose.pose.position.y
                ref_vel = pose.pose.position.z  # 速度存储在z中
                
                point_time = base_time + i * time_interval
                
                self.ref_traj_times.append(point_time)
                self.ref_path_x.append(x)
                self.ref_path_y.append(y)
                self.ref_vel_x_data.append(ref_vel)
                
    def actual_trajectory_callback(self, msg):
        """处理实际轨迹数据"""
        with self.lock:
            self.data_received['act_traj'] = True
            
            # 清除旧数据
            self.act_traj_times.clear()
            self.act_path_x.clear()
            self.act_path_y.clear()
            
            # 计算相对于第一次实际轨迹消息的时间
            trajectory_time = self.get_relative_time(msg, 'act_traj')
            
            # 提取轨迹点
            for pose in msg.poses:
                self.act_traj_times.append(trajectory_time)
                self.act_path_x.append(pose.pose.position.x)
                self.act_path_y.append(pose.pose.position.y)
                
    def odom_callback(self, msg):
        """处理里程计数据"""
        with self.lock:
            if not self.data_received['odom']:
                rospy.loginfo("[Plotter]: First odometry data received")
                self.data_received['odom'] = True
            
            # 计算相对于第一次里程计消息的时间
            current_time = self.get_relative_time(msg, 'odom')
            
            # 保存时间和数据
            self.odom_times.append(current_time)
            
            # 实际线速度和角速度
            act_vel_x = msg.twist.twist.linear.x
            act_vel_z = msg.twist.twist.angular.z
            
            self.act_vel_x_data.append(act_vel_x)
            self.act_vel_z_data.append(act_vel_z)
                
    def update_plots(self):
        """更新所有图表"""
        with self.lock:
            # 1. 更新轨迹图
            if self.ref_path_x and self.ref_path_y:
                self.ref_path_line.set_data(list(self.ref_path_x), list(self.ref_path_y))
            
            if self.act_path_x and self.act_path_y:
                self.act_path_line.set_data(list(self.act_path_x), list(self.act_path_y))
            
            # 自动调整轨迹图范围
            if self.ref_path_x or self.act_path_x:
                all_x = list(self.ref_path_x) + list(self.act_path_x)
                all_y = list(self.ref_path_y) + list(self.act_path_y)
                if all_x:
                    x_min, x_max = min(all_x), max(all_x)
                    y_min, y_max = min(all_y), max(all_y)
                    x_margin = (x_max - x_min) * 0.1 if x_max > x_min else 0.5
                    y_margin = (y_max - y_min) * 0.1 if y_max > y_min else 0.5
                    self.ax1.set_xlim(x_min - x_margin, x_max + x_margin)
                    self.ax1.set_ylim(y_min - y_margin, y_max + y_margin)
            
            # 2. 更新线速度图
            if self.odom_times and self.act_vel_x_data:
                act_time_list = list(self.odom_times)
                act_vel_x_list = list(self.act_vel_x_data)
                self.act_vel_x_line.set_data(act_time_list, act_vel_x_list)
            
            if self.ref_traj_times and self.ref_vel_x_data:
                ref_time_list = list(self.ref_traj_times)
                ref_vel_x_list = list(self.ref_vel_x_data)
                self.ref_vel_x_line.set_data(ref_time_list, ref_vel_x_list)
            
            # 调整线速度图范围
            if self.odom_times or self.ref_traj_times:
                all_times = []
                if self.odom_times: all_times.extend(list(self.odom_times))
                if self.ref_traj_times: all_times.extend(list(self.ref_traj_times))
                
                if all_times:
                    time_min, time_max = min(all_times), max(all_times)
                    if time_max > time_min:
                        time_margin = (time_max - time_min) * 0.05 if time_max > time_min else 0.5
                        self.ax2.set_xlim(time_min - time_margin, time_max + time_margin)
                    
                    # 获取所有速度数据
                    all_vel = []
                    if self.act_vel_x_data: all_vel.extend(list(self.act_vel_x_data))
                    if self.ref_vel_x_data: all_vel.extend(list(self.ref_vel_x_data))
                    
                    if all_vel:
                        vel_min, vel_max = min(all_vel), max(all_vel)
                        margin = (vel_max - vel_min) * 0.1 if vel_max > vel_min else 0.1
                        self.ax2.set_ylim(vel_min - margin, vel_max + margin)
            
            # 3. 更新角速度图
            if self.odom_times and self.act_vel_z_data:
                act_time_list = list(self.odom_times)
                act_vel_z_list = list(self.act_vel_z_data)
                self.act_vel_z_line.set_data(act_time_list, act_vel_z_list)
            
            # 调整角速度图范围
            if self.odom_times:
                time_min, time_max = min(self.odom_times), max(self.odom_times)
                if time_max > time_min:
                    time_margin = (time_max - time_min) * 0.05 if time_max > time_min else 0.5
                    self.ax3.set_xlim(time_min - time_margin, time_max + time_margin)
                
                if self.act_vel_z_data:
                    ang_min, ang_max = min(self.act_vel_z_data), max(self.act_vel_z_data)
                    margin = max(abs(ang_min), abs(ang_max)) * 0.1 if ang_max > ang_min else 0.1
                    self.ax3.set_ylim(ang_min - margin, ang_max + margin)
        
        # 自动保存图片
        current_time = time.time()
        if current_time - self.last_save_time > self.save_interval:
            self.save_plot_auto()
            self.last_save_time = current_time
        
    def save_plot_auto(self):
        """自动保存当前绘图"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"pid_plot_auto_{timestamp}.png"
        filepath = os.path.join(self.plot_dir, filename)
        
        try:
            self.fig.savefig(filepath, dpi=150, bbox_inches='tight')
            rospy.logdebug(f"[Plotter]: Plot auto-saved to: {filepath}")
        except Exception as e:
            rospy.logwarn(f"[Plotter]: Failed to auto-save plot: {e}")
        
    def save_final_plot(self):
        """保存最终图像"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        
        # 保存组合图
        filename = f"pid_plot_final_{timestamp}.png"
        filepath = os.path.join(self.plot_dir, filename)
        
        try:
            self.fig.savefig(filepath, dpi=300, bbox_inches='tight')
            rospy.loginfo(f"[Plotter]: Final plot saved to: {filepath}")
            
            # 单独保存三个子图
            self.save_individual_plots(timestamp)
            
        except Exception as e:
            rospy.logerr(f"[Plotter]: Failed to save plot: {e}")
    
    def save_individual_plots(self, timestamp):
        """单独保存三个子图"""
        try:
            # 1. 轨迹图
            fig1, ax1 = plt.subplots(figsize=(8, 6))
            if self.ref_path_x and self.ref_path_y:
                ax1.plot(self.ref_path_x, self.ref_path_y, 'b-', label='Reference', linewidth=2, alpha=0.8)
            if self.act_path_x and self.act_path_y:
                ax1.plot(self.act_path_x, self.act_path_y, 'r-', label='Actual', linewidth=1.5, alpha=0.8)
            
            ax1.set_title('Reference vs Actual Trajectory')
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Y [m]')
            ax1.grid(True, alpha=0.3)
            ax1.axis('equal')
            ax1.legend(loc='upper right')
            
            trajectory_file = f"trajectory_{timestamp}.png"
            trajectory_path = os.path.join(self.plot_dir, trajectory_file)
            fig1.savefig(trajectory_path, dpi=300, bbox_inches='tight')
            plt.close(fig1)
            
            # 2. 线速度图
            fig2, ax2 = plt.subplots(figsize=(8, 6))
            if self.odom_times and self.act_vel_x_data:
                ax2.plot(self.odom_times, self.act_vel_x_data, 'r-', label='Actual', linewidth=1.5, alpha=0.8)
            if self.ref_traj_times and self.ref_vel_x_data:
                ax2.plot(self.ref_traj_times, self.ref_vel_x_data, 'b-', label='Reference', linewidth=2, alpha=0.8)
            
            ax2.set_title('Linear Velocity Tracking')
            ax2.set_xlabel('Time [s]')
            ax2.set_ylabel('Velocity [m/s]')
            ax2.grid(True, alpha=0.3)
            ax2.legend(loc='upper right')
            
            linear_vel_file = f"linear_velocity_{timestamp}.png"
            linear_vel_path = os.path.join(self.plot_dir, linear_vel_file)
            fig2.savefig(linear_vel_path, dpi=300, bbox_inches='tight')
            plt.close(fig2)
            
            # 3. 角速度图
            fig3, ax3 = plt.subplots(figsize=(8, 6))
            if self.odom_times and self.act_vel_z_data:
                ax3.plot(self.odom_times, self.act_vel_z_data, 'b-', label='Actual', linewidth=1.5, alpha=0.8)
            
            ax3.set_title('Angular Velocity')
            ax3.set_xlabel('Time [s]')
            ax3.set_ylabel('Angular Velocity [rad/s]')
            ax3.grid(True, alpha=0.3)
            ax3.legend(loc='upper right')
            
            angular_vel_file = f"angular_velocity_{timestamp}.png"
            angular_vel_path = os.path.join(self.plot_dir, angular_vel_file)
            fig3.savefig(angular_vel_path, dpi=300, bbox_inches='tight')
            plt.close(fig3)
            
            rospy.loginfo(f"[Plotter]: Individual plots saved: {trajectory_file}, {linear_vel_file}, {angular_vel_file}")
            
        except Exception as e:
            rospy.logwarn(f"[Plotter]: Failed to save individual plots: {e}")
        
    def run(self):
        """主循环"""
        rospy.loginfo("[Plotter]: Starting plot update loop")
        
        try:
            while not rospy.is_shutdown():
                self.update_plots()
                rospy.sleep(1.0 / self.plot_update_rate)
                
        except KeyboardInterrupt:
            rospy.loginfo("[Plotter]: Plotter interrupted by user")
        except Exception as e:
            rospy.logerr(f"[Plotter]: Error in plotter: {e}")
        finally:
            # 保存最终图像
            self.save_final_plot()
            rospy.loginfo("[Plotter]: Plotter shutdown complete")

if __name__ == '__main__':
    try:
        plotter = Plotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass