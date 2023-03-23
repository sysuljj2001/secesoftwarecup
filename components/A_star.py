import math
import time
import matplotlib.pyplot as plt
import numpy as np
from typing import List
min_set = 10
show_animation = False  # 绘图

# 创建一个类
class Dijkstra:
    # 初始化
    def __init__(self, ox, oy, resolution, robot_radius, ang_step):
        '''A* 寻路器
        :param ox, oy: 障碍物 x, y 坐标列表
        :param resolution: 地图分辨率/栅格大小
        :param robot_radius: 机器人半径/轴距
        :param ang_step: 机器人离散转角步长，单位为弧度
        '''
        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None

        self.ang_step = ang_step
        self.resolution = resolution  # 网格大小(m)
        self.robot_radius = robot_radius  #
        self.calc_obstacle_map(ox, oy)  # 绘制栅格地图
        #self.motion = self.get_motion_model()  # 机器人路线权重

    # 构建节点，每个可行连续位置代表一个节点
    class Node:
        def __init__(self, rx, ry, x, y, theta, cost, parent_index):
            self.x = x  # 网格索引
            self.y = y
            self.rx = rx # 真实坐标
            self.ry = ry
            self.theta = theta # 节点角度
            self.cost = cost  # 路径值
            self.parent_index = parent_index  #网格父节点

        def __str__(self):
            return str(self.x) + ',' + str(self.y) + ',' + str(self.cost) + ',' + str(self.parent_index)

    # 寻找最优路径，网格起始坐标(sx,sy)，终点坐标（gx,gy），起点角度 theta
    def planning(self, sx, sy, theta, gx, gy):
        '''寻路
        :param sx, sy: 起始点坐标
        :param theta: 起始点机器人朝向，单位为弧度
        :param gx, gy: 终止点坐标
        '''
        # 节点初始化
        # 将已知的起点和终点坐标形式转化为节点类型，0代表路径权重，-1代表无父节点
        start_node = self.Node(sx, sy, self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), theta, 0.0, -1)
        # 终点
        goal_node = self.Node(gx, gy, self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0, 0.0, -1)
        # 保存入库节点和待计算节点
        open_set, closed_set = dict(), dict()
        # 先将起点入库，获取每个网格对应的key
        open_set[self.calc_index(start_node)] = start_node

        # 循环
        while 1:
            # 选择扩展点，添加了启发项，f(n)= g(n) + h(n)
            c_id = min(open_set,
                       key=lambda o: open_set[o].cost + \
                                     self.calc_heuristic(goal_node, open_set[o]))

            current = open_set[c_id]  # 从字典中取出该节点

            # 绘图
            """"""
            if show_animation:
                # 网格索引转换为真实坐标
                #print(current)
                plt.plot(self.calc_position(current.x, self.min_x),
                         self.calc_position(current.y, self.min_y), 'xc')
                plt.pause(0.0001)
            
            # 判断是否是终点，如果选出来的损失最小的点是终点
            if current.x == goal_node.x and current.y == goal_node.y \
               or math.hypot(current.rx - goal_node.rx, current.ry - goal_node.ry) <= 0.4:
                # 更新终点的父节点
                goal_node.cost = current.cost
                # 更新终点的损失
                goal_node.parent_index = current.parent_index
                break

            # 在外库中删除该最小损失点，把它入库
            del open_set[c_id]
            closed_set[c_id] = current

            # 遍历邻接节点
            for new_node in self.get_new_nodes(current):
                # 获取每个邻接节点的节点坐标，到起点的距离，父节点

                # 获取该邻居节点的key
                n_id = self.calc_index(new_node)

                # 如果该节点入库了，就看下一个
                if n_id in closed_set:
                    continue

                # 邻居节点是否超出范围了，是否在障碍物上
                if not self.verify_node(new_node):
                    continue

                # 如果该节点不在外库中，就作为一个新节点加入到外库
                if n_id not in open_set:
                    open_set[n_id] = new_node
                # 节点在外库中时
                else:
                    # 如果该点到起点的距离，要小于外库中该点的距离，就更新外库中的该点信息，更改路径
                    if new_node.cost <= open_set[n_id].cost:
                        open_set[n_id] = new_node

        # 找到终点
        rx, ry = self.calc_final_path(goal_node, closed_set)
        res = list(zip(rx, ry))
        res.reverse()
        return res

    # ------------------------------ #
    # A* 的启发函数
    # ------------------------------ #
    @staticmethod
    def calc_heuristic(n1, n2):  # n1终点，n2当前网格
        w = [0.99, 0.01]  # 单个启发函数的权重，如果有多个启发函数，权重可以设置的不一样
        d = w[0] * math.hypot(n1.x - n2.x, n1.y - n2.y)  # 当前网格和终点距离
        _gx, _gy = n1.rx - n2.rx, n1.ry - n2.ry
        if n1.x - n2.x == 0:
            k = w[1] * np.abs(np.rad2deg(np.sign(n1.ry - n2.ry) * np.pi / 2 - n2.theta))
        else: 
            k = w[1] * np.abs(np.rad2deg(n2.theta - np.arctan(_gy / _gx))) # 机器人朝向与机器人到终点的直线距离夹角
        return d + k

    # 机器人行走的方式，每次能向前方 7 个连续位置移动
    def get_new_nodes(self, node: Node):
        # [dx, dy, cost]
        # 2023/03/23 future: 路径曲率受最大转向角约束
        new_node = []
        for i in range(-4, 4):
            l = np.sqrt(2.2) * self.resolution
            angle = np.arctan(2 * self.robot_radius / l * np.sin(self.ang_step * i / 2))
            px, py = l * np.cos(node.theta), l * np.sin(node.theta)
            nx = node.rx + np.cos(angle) * px + np.sin(angle) * py
            ny = node.ry + np.cos(angle) * py - np.sin(angle) * px
            new_node.append(self.Node(nx, ny, 
                                      self.calc_xy_index(nx, self.min_x), 
                                      self.calc_xy_index(ny, self.min_y),
                                      node.theta - angle,
                                      1 + np.abs(angle) * 2, self.calc_index(node)))
        return new_node

    # 绘制栅格地图
    def calc_obstacle_map(self, ox, oy):
        # 地图边界坐标
        self.min_x = round(min(ox))  # 四舍五入取整
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        # 地图的x和y方向的栅格个数，长度/每个网格的长度=网格个数
        self.x_width = round((self.max_x - self.min_x) / self.resolution)  # x方向网格个数
        self.y_width = round((self.max_y - self.min_y) / self.resolution)  # y方向网格个数
        # 初始化地图，二维列表，每个网格的值为False
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
    """
        # 设置障碍物
        for ix in range(self.x_width):  # 遍历x方向的网格 [0:x_width]
            x = self.calc_position(ix, self.min_x)  # 根据网格索引计算x坐标位置
            for iy in range(self.y_width):  # 遍历y方向的网格 [0:y_width]
                y = self.calc_position(iy, self.min_y)  # 根据网格索引计算y坐标位置
                # 遍历障碍物坐标(实际坐标)
                for iox, ioy in zip(ox, oy):
                    # 计算障碍物和网格点之间的距离
                    d = math.hypot(iox - x, ioy - y)
                    # 膨胀障碍物，如果障碍物和网格之间的距离小，机器人无法通行，对障碍物膨胀
                    if d <= self.robot_radius:
                        # 将障碍物所在网格设置为True
                        self.obstacle_map[ix][iy] = True
                        break  # 每个障碍物膨胀一次就足够了
    """
    # 根据网格编号计算实际坐标
    def calc_position(self, index, minp):
        # minp代表起点坐标，左下x或左下y
        pos = minp + index * self.resolution  # 网格点左下左下坐标
        return pos

    # 位置坐标转为网格坐标
    def calc_xy_index(self, position, minp):
        # (目标位置坐标-起点坐标)/一个网格的长度==>目标位置的网格索引
        return round((position - minp) / self.resolution)

    # 给每个网格编号，得到每个网格的key
    def calc_index(self, node):
        # 从左到右增大，从下到上增大
        return node.y * self.x_width + node.x

    # 邻居节点是否超出范围
    def verify_node(self, node):
        # 根据网格坐标计算实际坐标
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)
        # 判断是否超出边界
        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False
        # 节点是否在障碍物上，障碍物标记为True
        if self.obstacle_map[node.x][node.y]:
            return False
        # 没超过就返回True
        return True

    # 计算路径, parent属性记录每个节点的父节点
    def calc_final_path(self, goal_node, closed_set):
        # 先存放终点坐标（真实坐标）
        rx = [self.calc_position(goal_node.x, self.min_x)]
        ry = [self.calc_position(goal_node.y, self.min_y)]
        # 获取终点的父节点索引
        parent_index = goal_node.parent_index
        # 起点的父节点==-1
        while parent_index != -1:
            n = closed_set[parent_index]  # 在入库中选择父节点
            rx.append(self.calc_position(n.x, self.min_x))  # 节点的x坐标
            ry.append(self.calc_position(n.y, self.min_y))  # 节点的y坐标
            parent_index = n.parent_index  # 节点的父节点索引

        return rx, ry

def main():
    # 设置起点和终点
    sx = 10.0
    sy = 20.0
    gx = 30
    gy = 10
    # 网格大小
    grid_size = 0.5
    # 机器人半径
    robot_radius = 0.45

    # 设置障碍物位置
    ox, oy = [], []
    for i in range(0, 100):    ox.append(i); oy.append(0.0)  # 下边界
    for i in range(0, 100):    ox.append(100.0); oy.append(i)  # 右边界
    for i in range(0, 100):    ox.append(i); oy.append(100.0)  # 上边界
    for i in range(0, 100):    ox.append(0.0); oy.append(i)  # 左边界
    #for i in range(20, 60):    ox.append(60.0); oy.append(i)  # 左围栏
    #for i in range(0, 40):      ox.append(40.0); oy.append(60 - i)  # 右围栏

    # 绘图
    if show_animation:
        plt.plot(gx, gy, '.k')  # 障碍物黑色
        plt.plot(sx, sy, 'og')  # 起点绿色
        plt.grid(True)
        plt.axis('equal')  # 坐标轴刻度间距等长

    # 实例化，传入障碍物，网格大小
    #dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
    # 求解路径，返回路径的 x 坐标和 y 坐标列表
    #t1=time.time()
    finder = Dijkstra([0, 50], [0, 50], grid_size, robot_radius, np.deg2rad(10))
    paths = finder.planning(sx, sy, np.deg2rad(-180), gx, gy)
    print(paths)
    #res = dijkstra.planning(sx, sy, gx, gy)#需实际校对
    #print(res)#绘图模块删掉后决策离散路线在10ms左右
    #rx, ry = dijkstra.planning(70, 70, 40, 50)
    #print(rx, ry)
    #rx, ry = dijkstra.planning(40, 50, 10, 90)
    #print(rx, ry)
    #rx, ry = dijkstra.planning(10, 90, np.around(86.6),np.around(45.3))
    #print(rx, ry)
    #t2=time.time()
    #print(t2-t1)
if __name__ == '__main__':
    main()
