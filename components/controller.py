'''
控制器模块（Controller）：用于控制机器人运动学动作和行为，如转向、加速
'''

import math
import logging
import numpy as np
from typing import List
from components.preprocess import DataLoader

def distance(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) + 0.001

def check_distance(p1, p2, boundary=0.5):
    return distance(p1, p2) < boundary

def glob_check(bot_infos: List, dis: float, bot_radius: float):
        '''全局碰撞预警
        :param dis: 预警距离阈值（虚拟围栏）
        '''
        for bot_1 in bot_infos:
            for bot_2 in bot_infos:
                if bot_1['id'] == bot_2['id']: continue
                if check_distance(bot_1['coord'], bot_2['coord'], dis + bot_radius * 2):
                    return True
        return False

def repulsive_force(dist, safe_distance, k=3):
    return k * (1/dist - 1/safe_distance) * (1/(dist**2))

def avoid_collision(pos, vel, ang_vel, 
                    ori, dt=0.1, safe_distance=2, max_speed=6.0, 
                    max_angular_velocity=np.pi):
    num_agents = len(pos)
    new_velocities = [np.array([v * np.cos(ori[i]), v * np.sin(ori[i])]) for i, v in enumerate(vel)]
    new_angular_velocities = ang_vel.copy()

    for i in range(num_agents):
        for j in range(i+1, num_agents):
            dist = np.linalg.norm(pos[j] - pos[i])

            # 如果距离小于安全距离，进行规避操作
            if dist < safe_distance:
                # 计算两智能体间的方向向量
                direction = (pos[j] - pos[i]) / dist
                force = repulsive_force(dist, safe_distance)
                # 为避免碰撞，智能体i和j分别调整速度和角速度
                new_velocities[i] -= direction * force * dt
                new_velocities[j] += direction * force * dt
                angle_i = ori[i]
                angle_j = ori[j]

                # 计算角度差
                angle_diff = (angle_j - angle_i) % (2 * np.pi)
                if angle_diff > np.pi:
                    angle_diff -= 2 * np.pi

                # 调整角速度
                if angle_diff > 0:
                    new_angular_velocities[i] -= max_angular_velocity  * dt
                    new_angular_velocities[j] += max_angular_velocity  * dt
                else:
                    new_angular_velocities[i] += max_angular_velocity  * dt
                    new_angular_velocities[j] -= max_angular_velocity * dt
            if dist < 1.2:
                # 已经撞在一起
                x_offset = np.abs(np.cos(np.arctan2(direction[1], direction[0])) * safe_distance)
                delta_x = np.abs(pos[i][0] - x_offset)
                new_velocities[i] *= -2
                new_velocities[j] *= 1.5
                if delta_x < x_offset:
                    new_angular_velocities[i] = 2
                    new_angular_velocities[j] = -2
                else:
                    new_angular_velocities[i] = 1.5
                    new_angular_velocities[j] = -1.5
                if np.linalg.norm(direction) < 0.5:
                    # 几乎同向
                    new_angular_velocities[i] *= -1

    new_velocities = [min(np.linalg.norm(v), max_speed) for v in new_velocities]
    return new_velocities, new_angular_velocities

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
    def __init__(self, bot_info: dict = None, paths: List = None, kp=[4, 3], ki=[0.0007, 0.0001], kd=[0.00005, 0.0002]) -> None:
        '''PID 控制器，输入一帧机器人位姿状态和当前帧行走路径集合，输出期望的线速度和角速度
        :param bot_info: 从地图数据中获取的机器人位姿
        :param paths: 机器人行走路径集合（有序点集）
        '''
        self.bot_info = bot_info
        self.paths = paths
        self.v_pid = PID(kp[0], ki[0], kd[0])
        self.w_pid = PID(kp[1], ki[1], kd[1])
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
        if np.abs(angle_to_goal - self.bot_info['p']) > np.pi:
            return np.pi * 2 + angle_to_goal - self.bot_info['p']
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
        self.v_pid.refresh()
        self.w_pid.refresh()

    def handle(self, dt):
        '''获取该机器人前往 dest 在当前帧所需的 forwad 和 rotate
        '''
        distance_to_goal = self.distance_to_goal()
        angle_to_goal = self.angle_to_goal()
        if distance_to_goal < 0.2:
            self.current_path_index += 1
            if self.current_path_index >= len(self.paths):
                return None

        linear_velocity = self.v_pid.update(distance_to_goal, dt)
        angular_velocity = self.w_pid.update(angle_to_goal, dt)
        if linear_velocity > 6: linear_velocity = 6
        elif linear_velocity < -2: linear_velocity = -2
        if angular_velocity > np.pi: angular_velocity = np.pi
        elif angular_velocity < -np.pi: angular_velocity = -np.pi

        ''''''
        if angle_to_goal >= np.pi:
            angular_velocity = 3
            linear_velocity = 2
        elif angle_to_goal <= -np.pi:
            angular_velocity = -3
            linear_velocity = 2
        elif angle_to_goal >= np.pi / 2:
            angular_velocity = 3
            linear_velocity = 1.8
        elif angle_to_goal <= -np.pi / 2:
            angular_velocity = -3
            linear_velocity = 1.8
        elif angle_to_goal > np.pi / 3:
            angular_velocity = 3
            linear_velocity = 2
        elif angle_to_goal < -np.pi /3:
            angular_velocity = -3
            linear_velocity = 2

        return linear_velocity, angular_velocity

if __name__=='__main__':
    pass


