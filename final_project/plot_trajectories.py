#!/usr/bin/env python2
# coding=utf-8

import matplotlib.pyplot as plt
import numpy as np
import os

def read_trajectory(filename):
    """读取轨迹文件"""
    if not os.path.exists(filename):
        print("File not found: {}".format(filename))
        return None
    
    points = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    x, y = map(float, line.split(','))
                    points.append([x, y])
                except ValueError:
                    continue
    
    return np.array(points) if points else None

def calculate_distances(actual_points, planned_points):
    """计算每个实际点到规划路径的最近距离（不使用scipy）"""
    distances = []
    for actual_point in actual_points:
        # 计算实际点到规划路径上所有点的距离
        min_dist = float('inf')
        for planned_point in planned_points:
            dist = np.sqrt((actual_point[0] - planned_point[0])**2 + 
                          (actual_point[1] - planned_point[1])**2)
            if dist < min_dist:
                min_dist = dist
        distances.append(min_dist)
    return distances

def plot_trajectories():
    # 文件路径
    planned_file = "/home/huangq256/docker2/final_project/planned_path.txt"
    actual_file = "/home/huangq256/docker2/final_project/actual_path.txt"
    
    # 读取数据
    planned_points = read_trajectory(planned_file)
    actual_points = read_trajectory(actual_file)
    
    if planned_points is None and actual_points is None:
        print("No trajectory data found!")
        return
    
    # 创建图形 - 使用2行1列的布局（删除速度剖面后）
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # 子图1：轨迹对比
    ax1 = axes[0, 0]
    if planned_points is not None:
        ax1.plot(planned_points[:, 0], planned_points[:, 1], 'b-', 
                 linewidth=2, label='Planned Path', alpha=0.8)
    if actual_points is not None:
        ax1.plot(actual_points[:, 0], actual_points[:, 1], 'r--', 
                 linewidth=2, label='Actual Path', alpha=0.8)
    
    # 标记起点和终点
    if planned_points is not None:
        ax1.scatter(planned_points[0, 0], planned_points[0, 1], 
                   c='green', s=200, marker='o', label='Start', zorder=5)
        ax1.scatter(planned_points[-1, 0], planned_points[-1, 1], 
                   c='red', s=200, marker='*', label='Goal', zorder=5)
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Planned vs Actual Trajectory')
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 子图2：轨迹偏差分析
    ax2 = axes[0, 1]
    if planned_points is not None and actual_points is not None:
        distances = calculate_distances(actual_points, planned_points)
        
        ax2.plot(distances, 'g-', linewidth=2)
        ax2.set_xlabel('Time Step')
        ax2.set_ylabel('Tracking Error (m)')
        ax2.set_title('Cross-track Error Over Time')
        ax2.grid(True, alpha=0.3)
        
        # 统计信息
        mean_error = np.mean(distances)
        max_error = np.max(distances)
        rmse = np.sqrt(np.mean(np.array(distances)**2))
        
        stats_text = 'Mean: {:.4f}m\nMax: {:.4f}m\nRMSE: {:.4f}m'.format(
            mean_error, max_error, rmse)
        ax2.text(0.98, 0.98, stats_text, transform=ax2.transAxes,
                 verticalalignment='top', horizontalalignment='right',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # 子图3：误差分布直方图
    ax3 = axes[1, 0]
    if planned_points is not None and actual_points is not None:
        distances = calculate_distances(actual_points, planned_points)
        
        ax3.hist(distances, bins=30, edgecolor='black', alpha=0.7)
        ax3.set_xlabel('Tracking Error (m)')
        ax3.set_ylabel('Frequency')
        ax3.set_title('Error Distribution Histogram')
        ax3.grid(True, alpha=0.3)
    
    # 子图4：留空或显示统计信息
    ax4 = axes[1, 1]
    ax4.axis('off')  # 关闭坐标轴
    if planned_points is not None and actual_points is not None:
        distances = calculate_distances(actual_points, planned_points)
        mean_error = np.mean(distances)
        max_error = np.max(distances)
        rmse = np.sqrt(np.mean(np.array(distances)**2))
        total_points = len(actual_points)
        
        summary_text = ('Trajectory Tracking Summary\n\n' +
                      'Total Points: {}\n'.format(total_points) +
                      'Mean Error: {:.4f} m\n'.format(mean_error) +
                      'Max Error: {:.4f} m\n'.format(max_error) +
                      'RMSE: {:.4f} m\n'.format(rmse))
        
        ax4.text(0.5, 0.5, summary_text, transform=ax4.transAxes,
                 verticalalignment='center', horizontalalignment='center',
                 bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5),
                 fontsize=12)
        ax4.set_title('Summary Statistics')
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # 检查是否安装了必要的库
    try:
        import matplotlib
        plot_trajectories()
    except ImportError:
        print("Please install required packages:")
        print("sudo apt-get install python-matplotlib python-numpy")