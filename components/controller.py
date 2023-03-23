'''
控制器模块（Controller）：用于控制机器人运动学动作和行为，如转向、加速
'''

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree #浮点数路线存储数据结构
from celluloid import Camera #动图输出
from typing import List

class RVO_Contorller():
    def __init__(self) -> None:
        '''RVO 控制器，用于避障
        '''
        pass

class PID:
    def __init__(self, kp, ki, kd):
        '''PID 计算器
        '''
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0
    
    def refresh(self):
        self.last_error = 0
        self.integral = 0
    
    def update(self, error, dt):
        derivative = (error - self.last_error) / dt
        self.integral += error * dt
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class PID_Controller():
    def __init__(self, bot_info: dict = None, paths: List = None, kp=4, ki=0.0007, kd=0.00005) -> None:
        '''PID 控制器，输入一帧机器人位姿状态和当前帧行走路径集合，输出期望的线速度和角速度
        :param bot_info: 从地图数据中获取的机器人位姿
        :param paths: 机器人行走路径集合（有序点集）
        '''
        self.bot_info = bot_info
        self.paths = paths
        self.pid = PID(kp, ki, kd)
        self.current_path_index = 0

    def distance_to_goal(self):
        '''距离误差
        '''
        if self.current_path_index >= len(self.paths):
            return 0
        goal = self.paths[self.current_path_index]
        dx = goal[0] - self.bot_info['coord'][0]
        dy = goal[1] - self.bot_info['coord'][1]
        return math.sqrt(dx**2 + dy**2)

    def angle_to_goal(self):
        '''角度误差
        '''
        if self.current_path_index >= len(self.paths):
            return 0
        goal = self.paths[self.current_path_index]
        dx = goal[0] - self.bot_info['coord'][0]
        dy = goal[1] - self.bot_info['coord'][1]
        angle_to_goal = math.atan2(dy, dx)
        return angle_to_goal - self.bot_info['p']
    
    def update_path(self, paths):
        '''更新移动路径
        '''
        self.paths = paths
        self.current_path_index = 0

    def update_bot(self, bot_info: dict):
        '''更新机器人位姿状态
        '''
        self.bot_info = bot_info
    
    def refresh(self):
        '''重置pid控制器
        '''
        self.pid.refresh()

    def handle(self, dt):
        '''获取该机器人前往 dest 在当前帧所需的 forwad 和 rotate
        '''
        distance_to_goal = self.distance_to_goal()
        angle_to_goal = self.angle_to_goal()

        if distance_to_goal < 0.5:
            self.current_path_index += 1
            if self.current_path_index >= len(self.paths):
                return None

        linear_velocity = self.pid.update(distance_to_goal, dt)
        angular_velocity = self.pid.update(angle_to_goal, dt)
        if linear_velocity > 6: linear_velocity = 6
        elif linear_velocity < -2: linear_velocity = -2
        if angular_velocity > 3:angular_velocity = 3
        elif angular_velocity < -3:angular_velocity = -3

        ''''''
        if np.abs(angle_to_goal) >= np.pi / 2:
            angular_velocity = 3
            linear_velocity = 1
        elif angle_to_goal > np.pi/3 :
            angular_velocity *= 1.5
            linear_velocity = 1
        elif angle_to_goal < -np.pi/3 :
            angular_velocity *= 1.5
            linear_velocity = -1

        return linear_velocity, angular_velocity

if __name__=='__main__':
    pass


