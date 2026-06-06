import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random
import math
import time
from matplotlib.animation import FuncAnimation

class RRT:
    def __init__(self, map_size=(500, 500), start=None, goal=None, obstacle_density=0.2, 
                 step_size=20, max_iterations=5000, goal_sample_rate=0.1, animation=True,
                 ensure_connectivity=True):
        """
        初始化RRT算法
        
        参数:
        map_size: 地图尺寸 (width, height)
        start: 起点坐标 (x, y)
        goal: 终点坐标 (x, y)
        obstacle_density: 障碍物密度 (0-1之间)
        step_size: 每次扩展的步长
        max_iterations: 最大迭代次数
        goal_sample_rate: 采样时选择目标的概率
        animation: 是否显示动画
        ensure_connectivity: 确保起点和终点之间有可达路径
        """
        self.width, self.height = map_size
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.goal_sample_rate = goal_sample_rate
        self.animation = animation
        self.ensure_connectivity = ensure_connectivity
        
        # 设置起点和终点
        if start is None:
            self.start = (50, 50)
        else:
            self.start = start
            
        if goal is None:
            self.goal = (self.width-50, self.height-50)
        else:
            self.goal = goal
        
        # 生成随机地图
        self.grid_map = self.generate_random_map(obstacle_density)
        
        # 确保起点和终点不在障碍物上
        while self.is_collision(self.start[0], self.start[1]):
            self.start = self.get_random_free_position()
        while self.is_collision(self.goal[0], self.goal[1]):
            self.goal = self.get_random_free_position()
        
        # 如果启用连通性检查，确保起点和终点之间有通路
        if self.ensure_connectivity:
            self.ensure_start_goal_connectivity()
        
        # 初始化树
        self.node_list = [self.start]  # 节点列表
        self.parent_list = [-1]  # 父节点索引列表
        self.path = None
        self.found_goal = False
        
        # 可视化设置
        if self.animation:
            self.setup_visualization()
    
    def generate_random_map(self, obstacle_density):
        """生成随机栅格地图，减少障碍物数量"""
        grid_map = np.zeros((self.height, self.width), dtype=np.uint8)  # 0表示free
        
        # 计算障碍物数量，大幅减少
        num_obstacles = int(self.width * self.height * obstacle_density / 100)  # 减少分母，减少障碍物数量
        
        print(f"生成 {num_obstacles} 个障碍物")
        
        for _ in range(num_obstacles):
            # 随机生成障碍物形状和位置
            obs_type = random.choice(['rectangle', 'circle'])
            
            if obs_type == 'rectangle':
                # 矩形障碍物，尺寸减小
                obs_width = random.randint(10, 30)  # 减小最大尺寸
                obs_height = random.randint(10, 30)
                x = random.randint(0, self.width - obs_width)
                y = random.randint(0, self.height - obs_height)
                
                # 设置障碍物区域为1
                grid_map[y:y+obs_height, x:x+obs_width] = 1
                
            else:  # 圆形障碍物
                radius = random.randint(8, 20)  # 减小半径
                cx = random.randint(radius, self.width - radius)
                cy = random.randint(radius, self.height - radius)
                
                # 创建圆形障碍物
                for i in range(max(0, cy-radius), min(self.height, cy+radius+1)):
                    for j in range(max(0, cx-radius), min(self.width, cx+radius+1)):
                        if (i-cy)**2 + (j-cx)**2 <= radius**2:
                            grid_map[i, j] = 1
        
        # 添加薄边界障碍物
        border_width = 3
        grid_map[:border_width, :] = 1  # 上边界
        grid_map[-border_width:, :] = 1  # 下边界
        grid_map[:, :border_width] = 1  # 左边界
        grid_map[:, -border_width:] = 1  # 右边界
        
        # 计算障碍物覆盖率
        obstacle_ratio = np.sum(grid_map) / (self.width * self.height)
        print(f"障碍物覆盖率: {obstacle_ratio:.2%}")
        
        return grid_map
    
    def ensure_start_goal_connectivity(self):
        """确保起点和终点之间有可通行的直线路径"""
        # 尝试在起点和终点之间创建一条无碰撞的通道
        x1, y1 = self.start
        x2, y2 = self.goal
        
        # 计算两点之间的距离
        dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        
        # 如果距离太远，创建一个中间通道
        if dist > 200:
            # 在起点和终点之间创建一个中间点
            mid_x = (x1 + x2) // 2
            mid_y = (y1 + y2) // 2
            
            # 在中间点周围创建一个无碰撞区域
            clearance_radius = 50
            for i in range(max(0, mid_y-clearance_radius), min(self.height, mid_y+clearance_radius+1)):
                for j in range(max(0, mid_x-clearance_radius), min(self.width, mid_x+clearance_radius+1)):
                    if math.sqrt((i-mid_y)**2 + (j-mid_x)**2) <= clearance_radius:
                        self.grid_map[i, j] = 0  # 清除障碍物
    
    def get_random_free_position(self):
        """获取一个空闲位置的随机点"""
        for _ in range(1000):  # 最多尝试1000次
            x = random.randint(0, self.width-1)
            y = random.randint(0, self.height-1)
            if not self.is_collision(x, y):
                return (x, y)
        # 如果找不到空闲位置，返回地图中心
        return (self.width//2, self.height//2)
    
    def is_collision(self, x, y):
        """检查点(x,y)是否在障碍物上"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return True
        return self.grid_map[int(y), int(x)] == 1
    
    def is_collision_line(self, x1, y1, x2, y2, num_points=20):
        """检查两点之间的线段是否与障碍物碰撞"""
        # 检查多个中间点
        for i in range(num_points+1):
            t = i / num_points
            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t
            if self.is_collision(x, y):
                return True
        return False
    
    def get_random_point(self):
        """随机采样点"""
        if random.random() < self.goal_sample_rate:
            return self.goal
        else:
            return (random.randint(0, self.width-1), random.randint(0, self.height-1))
    
    def get_nearest_node(self, point):
        """找到距离随机点最近的节点"""
        min_dist = float('inf')
        nearest_index = 0
        
        for i, node in enumerate(self.node_list):
            dist = math.sqrt((node[0]-point[0])**2 + (node[1]-point[1])**2)
            if dist < min_dist:
                min_dist = dist
                nearest_index = i
                
        return nearest_index
    
    def steer(self, from_node, to_point):
        """从from_node向to_point方向扩展步长"""
        # 计算方向向量
        dx = to_point[0] - from_node[0]
        dy = to_point[1] - from_node[1]
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < self.step_size:
            return to_point
        
        # 计算新节点位置
        scale = self.step_size / dist
        new_x = from_node[0] + dx * scale
        new_y = from_node[1] + dy * scale
        
        return (new_x, new_y)
    
    def setup_visualization(self):
        """设置可视化环境"""
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(0, self.height)
        self.ax.set_aspect('equal')
        
        # 绘制地图
        self.ax.imshow(self.grid_map, cmap='gray_r', origin='lower', 
                      extent=[0, self.width, 0, self.height], alpha=0.3)
        
        # 绘制起点和终点
        start_circle = patches.Circle(self.start, 10, color='green', label='Start')
        goal_circle = patches.Circle(self.goal, 10, color='red', label='Goal')
        self.ax.add_patch(start_circle)
        self.ax.add_patch(goal_circle)
        
        # 添加文本
        self.ax.text(self.start[0], self.start[1]+15, 'Start', 
                    fontsize=12, ha='center', color='green')
        self.ax.text(self.goal[0], self.goal[1]+15, 'Goal', 
                    fontsize=12, ha='center', color='red')
        
        # 初始化可视化元素
        self.tree_line, = self.ax.plot([], [], 'b-', linewidth=1, alpha=0.6)
        self.path_line, = self.ax.plot([], [], 'r-', linewidth=3, label='Path')
        self.current_node_point, = self.ax.plot([], [], 'go', markersize=6, alpha=0.7)
        self.random_point_point, = self.ax.plot([], [], 'mo', markersize=6, alpha=0.7)
        
        self.ax.legend()
        self.ax.set_title("RRT Path Planning")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        
        plt.ion()  # 启用交互模式
    
    def update_visualization(self, nearest_node, random_point, new_node, iteration):
        """更新可视化"""
        if not self.animation:
            return
            
        # 更新树
        tree_x = []
        tree_y = []
        for i, node in enumerate(self.node_list):
            parent_idx = self.parent_list[i]
            if parent_idx >= 0:
                parent = self.node_list[parent_idx]
                tree_x.extend([parent[0], node[0], None])
                tree_y.extend([parent[1], node[1], None])
        
        self.tree_line.set_data(tree_x, tree_y)
        
        # 更新当前节点和随机点
        if new_node:
            self.current_node_point.set_data([new_node[0]], [new_node[1]])
        if random_point:
            self.random_point_point.set_data([random_point[0]], [random_point[1]])
        
        # 更新路径
        if self.found_goal and self.path:
            path_x = [p[0] for p in self.path]
            path_y = [p[1] for p in self.path]
            self.path_line.set_data(path_x, path_y)
        
        # 更新标题
        self.ax.set_title(f"RRT Path Planning - Iteration: {iteration}")
        
        # 绘制
        plt.draw()
        plt.pause(0.001)
    
    def find_path(self):
        """回溯找到从起点到终点的路径"""
        if not self.found_goal:
            return []
        
        # 从终点开始回溯
        path = []
        current_index = len(self.node_list) - 1
        
        while current_index != -1:
            path.append(self.node_list[current_index])
            current_index = self.parent_list[current_index]
        
        path.reverse()
        return path
    
    def plan(self):
        """执行RRT路径规划"""
        print("开始RRT路径规划...")
        print(f"起点: {self.start}")
        print(f"终点: {self.goal}")
        print(f"地图大小: {self.width}x{self.height}")
        print(f"最大迭代次数: {self.max_iterations}")
        print(f"步长: {self.step_size}")
        
        start_time = time.time()
        
        for iteration in range(self.max_iterations):
            # 1. 随机采样
            random_point = self.get_random_point()
            
            # 2. 找到最近的节点
            nearest_index = self.get_nearest_node(random_point)
            nearest_node = self.node_list[nearest_index]
            
            # 3. 扩展新节点
            new_node = self.steer(nearest_node, random_point)
            
            # 4. 碰撞检测
            if not self.is_collision_line(nearest_node[0], nearest_node[1], 
                                         new_node[0], new_node[1]):
                # 5. 添加到树中
                self.node_list.append(new_node)
                self.parent_list.append(nearest_index)
                
                # 6. 检查是否到达目标
                dist_to_goal = math.sqrt((new_node[0]-self.goal[0])**2 + 
                                        (new_node[1]-self.goal[1])**2)
                
                if dist_to_goal <= self.step_size and not self.is_collision_line(
                    new_node[0], new_node[1], self.goal[0], self.goal[1]):
                    
                    # 到达目标，添加到树中
                    self.node_list.append(self.goal)
                    self.parent_list.append(len(self.node_list)-2)
                    self.found_goal = True
                    
                    # 找到路径
                    self.path = self.find_path()
                    
                    print(f"找到路径! 迭代次数: {iteration+1}")
                    print(f"路径长度: {len(self.path)} 个节点")
                    print(f"路径总距离: {self.calculate_path_length(self.path):.2f}")
                    
                    # 更新可视化
                    self.update_visualization(nearest_node, random_point, new_node, iteration+1)
                    
                    if self.animation:
                        plt.ioff()  # 关闭交互模式
                        self.show_final_result()
                    
                    end_time = time.time()
                    print(f"总耗时: {end_time-start_time:.2f} 秒")
                    
                    return self.path
                
                # 更新可视化
                if iteration % 3 == 0:  # 每3次迭代更新一次可视化
                    self.update_visualization(nearest_node, random_point, new_node, iteration+1)
        
        # 如果达到最大迭代次数仍未找到路径
        end_time = time.time()
        print(f"未找到路径! 已达到最大迭代次数: {self.max_iterations}")
        print(f"总耗时: {end_time-start_time:.2f} 秒")
        
        if self.animation:
            plt.ioff()  # 关闭交互模式
            plt.show()
        
        return []
    
    def calculate_path_length(self, path):
        """计算路径长度"""
        if len(path) < 2:
            return 0
        
        length = 0
        for i in range(len(path)-1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            length += math.sqrt(dx**2 + dy**2)
        
        return length
    
    def show_final_result(self):
        """显示最终结果"""
        # 创建新的图形显示最终结果
        fig_final, ax_final = plt.subplots(figsize=(10, 10))
        ax_final.set_xlim(0, self.width)
        ax_final.set_ylim(0, self.height)
        ax_final.set_aspect('equal')
        
        # 绘制地图
        ax_final.imshow(self.grid_map, cmap='gray_r', origin='lower', 
                       extent=[0, self.width, 0, self.height], alpha=0.3)
        
        # 绘制起点和终点
        start_circle = patches.Circle(self.start, 10, color='green', label='Start')
        goal_circle = patches.Circle(self.goal, 10, color='red', label='Goal')
        ax_final.add_patch(start_circle)
        ax_final.add_patch(goal_circle)
        
        # 绘制树
        for i, node in enumerate(self.node_list):
            parent_idx = self.parent_list[i]
            if parent_idx >= 0:
                parent = self.node_list[parent_idx]
                ax_final.plot([parent[0], node[0]], [parent[1], node[1]], 
                            'b-', linewidth=1, alpha=0.3)
        
        # 绘制路径
        if self.path and len(self.path) > 1:
            path_x = [p[0] for p in self.path]
            path_y = [p[1] for p in self.path]
            ax_final.plot(path_x, path_y, 'r-', linewidth=3, label='Path')
        
        # 添加图例和标题
        ax_final.legend()
        if self.found_goal:
            ax_final.set_title(f"RRT Path Planning - Final Result\nNodes: {len(self.node_list)}, Path Length: {self.calculate_path_length(self.path):.2f}")
        else:
            ax_final.set_title(f"RRT Path Planning - No Path Found\nNodes: {len(self.node_list)}")
        ax_final.set_xlabel("X")
        ax_final.set_ylabel("Y")
        
        plt.tight_layout()
        plt.show()


def main():
    """主函数"""
    # 参数设置
    map_size = (500, 500)  # 地图大小
    start = (50, 50)       # 起点
    goal = (450, 450)      # 终点
    
    # 创建RRT规划器
    rrt = RRT(
        map_size=map_size,
        start=start,
        goal=goal,
        obstacle_density=0.02,  # 大幅降低障碍物密度
        step_size=25,           # 增加步长
        max_iterations=3000,    # 最大迭代次数
        goal_sample_rate=0.55,  # 增加目标采样率
        animation=True,         # 是否显示动画
        ensure_connectivity=True  # 确保连通性
    )
    
    # 执行路径规划
    path = rrt.plan()
    
    # 打印结果
    if path:
        print("\n路径节点:")
        for i, point in enumerate(path):
            print(f"  {i+1}: ({point[0]:.1f}, {point[1]:.1f})")
    else:
        print("未找到路径")


if __name__ == "__main__":
    main()