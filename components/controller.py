'''
控制器模块（Controller）：用于控制机器人运动学动作和行为，如转向、加速
'''

import math
import logging
import numpy as np
from typing import List
from components.preprocess import DataLoader

class RVO:
    def __init__(self) -> None:
        pass

# 2023/03/25 future: 改成对部分机器人也可用（在 main 中也要适配）

class RVO_Contorller():
    def __init__(self, bot_radius: float, end_time: float = 2, max_v: float = 6) -> None:
        '''RVO 控制器，用于避障
        :param map_status: 地图状态，与 preprocess.Dataloader 定义一致
        :param paths: 当前各个机器人正在走的路径，有 bot_num 条
        '''
        self.bot_r = bot_radius
        self.bots = []
        self.targets = []
        self.t = 0
        self.freeze = False
        self.end_time = end_time
        self.max_v = max_v
        self.cache = { 'v': 0, 'w': 0 }

    def glob_check(self, dis: float):
        '''全局碰撞预警
        :param dis: 预警距离阈值（虚拟围栏）
        '''
        for bot_1 in self.bots:
            for bot_2 in self.bots:
                if bot_1['id'] == bot_2['id']: continue
                if self.check_distance(bot_1['coord'], bot_2['coord'], dis + self.bot_r * 2):
                    return True
        return False

    def feedback(self, bot_id: int):
        return self.cache['v'][bot_id], self.cache['w'][bot_id]

    @staticmethod
    def distance(p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) + 0.001

    @staticmethod
    def check_distance(p1, p2, boundary=0.5):
        return RVO_Contorller.distance(p1, p2) < boundary

    def get_desire_vel(self, max_v):
        desire_V = []
        for i in range(len(self.bots)):
            differ_x = [self.targets[i][j] - self.bots[i]['coord'][j] for j in range(2)]
            norm = self.distance(differ_x, [0, 0])
            norm_differ_x = [differ_x[j] * max_v / norm for j in range(2)]
            desire_V.append(norm_differ_x[:])
            if self.check_distance(self.bots[i]['coord'], self.targets[i], 0.1):
                desire_V[i][0] = 0
                desire_V[i][1] = 0
        return desire_V
    
    def in_between(self, theta_right, theta_dif, theta_left):
        if abs(theta_right - theta_left) <= np.pi:
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        else:
            if (theta_left < 0) and (theta_right > 0):
                theta_left += 2 * np.pi
                if theta_dif < 0:
                    theta_dif += 2 * np.pi
                if theta_right <= theta_dif <= theta_left:
                    return True
                else:
                    return False
            if (theta_left > 0) and (theta_right < 0):
                theta_right += 2 * np.pi
                if theta_dif < 0:
                    theta_dif += 2 * np.pi
                if theta_left <= theta_dif <= theta_right:
                    return True
                else:
                    return False
    
    def intersect(self, pA, vA, RVO_BA_all):
        norm_v = self.distance(vA, [0, 0])
        suitable_V = []
        unsuitable_V = []
        for theta in np.arange(0, 2 * np.pi, 0.1):
            for rad in np.arange(0.02, norm_v + 0.02, norm_v / 5.0):
                new_v = [rad * np.cos(theta), rad * np.sin(theta)]
                suit = True
                for RVO_BA in RVO_BA_all:
                    p_0 = RVO_BA[0]
                    left = RVO_BA[1]
                    right = RVO_BA[2]
                    dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]
                    theta_dif = np.arctan2(dif[1], dif[0])
                    theta_right = np.arctan2(right[1], right[0])
                    theta_left = np.arctan2(left[1], left[0])
                    if self.in_between(theta_right, theta_dif, theta_left):
                        suit = False
                        break
                if suit:
                    suitable_V.append(new_v)
                else:
                    unsuitable_V.append(new_v)
        new_v = vA[:]
        suit = True
        for RVO_BA in RVO_BA_all:
            p_0 = RVO_BA[0]
            left = RVO_BA[1]
            right = RVO_BA[2]
            dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]
            theta_dif = np.arctan2(dif[1], dif[0])
            theta_right = np.arctan2(right[1], right[0])
            theta_left = np.arctan2(left[1], left[0])
            if self.in_between(theta_right, theta_dif, theta_left):
                suit = False
                break
        if suit:
            suitable_V.append(new_v)
        else:
            unsuitable_V.append(new_v)

        if suitable_V:
            vA_post = min(suitable_V, key=lambda v: self.distance(v, vA))
            new_v = vA_post[:]
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]
                theta_dif = np.arctan2(dif[1], dif[0])
                theta_right = np.arctan2(right[1], right[0])
                theta_left = np.arctan2(left[1], left[0])
        else:
            tc_V = dict()
            for unsuit_v in unsuitable_V:
                tc_V[tuple(unsuit_v)] = 0
                tc = []
                for RVO_BA in RVO_BA_all:
                    p_0 = RVO_BA[0]
                    left = RVO_BA[1]
                    right = RVO_BA[2]
                    dist = RVO_BA[3]
                    rad = RVO_BA[4]
                    dif = [unsuit_v[0] + pA[0] - p_0[0], unsuit_v[1] + pA[1] - p_0[1]]
                    theta_dif = np.arctan2(dif[1], dif[0])
                    theta_right = np.arctan2(right[1], right[0])
                    theta_left = np.arctan2(left[1], left[0])
                    if self.in_between(theta_right, theta_dif, theta_left):
                        small_theta = abs(theta_dif - 0.5 * (theta_left + theta_right))
                        if abs(dist * np.sin(small_theta)) >= rad:
                            rad = abs(dist * np.sin(small_theta))
                        big_theta = np.arcsin(abs(dist * np.sin(small_theta)) / rad)
                        dist_tg = abs(dist * np.cos(small_theta)) - abs(rad * np.cos(big_theta))
                        if dist_tg < 0:
                            dist_tg = 0
                        tc_v = dist_tg / self.distance(dif, [0, 0])
                        tc.append(tc_v)
                tc_V[tuple(unsuit_v)] = min(tc) + 0.001
            WT = 0.2
            vA_post = min(unsuitable_V, key=lambda v: ((WT / tc_V[tuple(v)]) + self.distance(v, vA)))
        return vA_post

    def refresh(self):
        self.t = 0

    def update_bots(self, map_status: DataLoader):
        self.bots = map_status.bots

    def update_targets(self, map_status: DataLoader, paths: List):
        self.targets = [map_status.tables[pair['table_id']]['coord'] for pair in paths]

    def RVO_update(self, desire_V):
        V_current = [bot['v'] for bot in self.bots]
        Robot_Radius = self.bot_r + 0.1
        V_opt = list(V_current)
        X = [bot['coord'] for bot in self.bots]
        for i in range(len(X)):
            vA = [V_current[i][0], V_current[i][1]]
            pA = [X[i][0], X[i][1]]
            RVO_BA_all = []
            for j in range(len(X)):
                if i != j:
                    vB = [V_current[j][0], V_current[j][1]]
                    pB = [X[j][0], X[j][1]]
                    transl_vB_vA = [pA[0] + 0.5 * (vB[0] + vA[0]), pA[1] + 0.5 * (vB[1] + vA[1])]
                    dist_BA = self.distance(pA, pB)
                    theta_BA = np.arctan2(pB[1] - pA[1], pB[0] - pA[0])
                    if 2 * Robot_Radius > dist_BA:
                        dist_BA = 2 * Robot_Radius
                    theta_BAort = np.arcsin(2 * Robot_Radius / dist_BA)
                    theta_ort_left = theta_BA + theta_BAort
                    bound_left = [np.cos(theta_ort_left), np.sin(theta_ort_left)]
                    theta_ort_right = theta_BA - theta_BAort
                    bound_right = [np.cos(theta_ort_right), np.sin(theta_ort_right)]
                    RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2 * Robot_Radius]
                    RVO_BA_all.append(RVO_BA)
            vA_post = self.intersect(pA, desire_V[i], RVO_BA_all)
            V_opt[i] = vA_post[:]
        return V_opt

    def check_time(self, dt):
        self.t += dt
        if self.t >= self.end_time:
            return True
        return False

    def handle(self, dt):
        if self.check_time(dt):
            return None
        des_v = self.get_desire_vel(self.max_v)
        opt_v = self.RVO_update(des_v)
        v_list = []
        w_list = []
        for j in range(len(self.bots)):
            theta = np.arctan2(opt_v[j][1], opt_v[j][0])
            liner_v = np.sqrt(opt_v[j][0] ** 2 + opt_v[j][1] ** 2)
            v_list.append(liner_v)
            w = (theta - self.bots[j]['p']) #弧度制配合
            bot_c, bot_p = self.bots[j]['coord'], self.bots[j]['p']
            #logging.info(f'bot {j} at {bot_c} face {np.rad2deg(bot_p)}, to: {self.targets[j]}, delta: {np.rad2deg(w)}')
            if w > np.pi:
                w = np.pi
            elif w < -np.pi:
                w = -np.pi
            w_list.append(w)
        self.cache = {'v': v_list, 'w': w_list}
        return {'v': v_list, 'w': w_list}


def distance(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) + 0.001

def check_distance(p1, p2, boundary=0.5):
    return RVO_Contorller.distance(p1, p2) < boundary

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

            # 计算两智能体之间的距离
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
                new_angular_velocities[i] = -3
                new_angular_velocities[j] = 3
                new_velocities[i] = -1
                new_velocities[j] = 1

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
        self.pid.refresh()

    def handle(self, dt):
        '''获取该机器人前往 dest 在当前帧所需的 forwad 和 rotate
        '''
        distance_to_goal = self.distance_to_goal()
        angle_to_goal = self.angle_to_goal()
        if distance_to_goal < 0.2:
            self.current_path_index += 1
            if self.current_path_index >= len(self.paths):
                return None

        linear_velocity = self.pid.update(distance_to_goal, dt)
        angular_velocity = self.pid.update(angle_to_goal, dt)
        if linear_velocity > 6: linear_velocity = 6
        elif linear_velocity < -2: linear_velocity = -2
        if angular_velocity > 3: angular_velocity = 3
        elif angular_velocity < -3: angular_velocity = -3

        ''''''
        if angle_to_goal >= np.pi:
            angular_velocity = 3
            linear_velocity *= 0.7
        elif angle_to_goal <= -np.pi:
            angular_velocity = 3
            linear_velocity *= 0.7
        elif angle_to_goal >= np.pi / 2:
            angular_velocity = 3
            linear_velocity = 1
        elif angle_to_goal <= -np.pi / 2:
            angular_velocity = 3
            linear_velocity = 1
        elif angle_to_goal > np.pi/3 :
            angular_velocity = 3
            linear_velocity = 1.5
        elif angle_to_goal < -np.pi/3 :
            angular_velocity = 3
            linear_velocity = 1.5

        return linear_velocity, angular_velocity

if __name__=='__main__':
    pass


