from RVO import RVO_update, calcu_desire_V, PI
#from vis import visualize_traj_dynamic
from numpy.random import random
from numpy import arctan2, sqrt

# ------------------------------
# define WorkStation model
ws_model = dict()
# robot radius
ws_model['robot_radius'] = 0.53
total_time = 15
V_max = 6
step = 0.015
#ws_model['circular_obstacles'] = []
#ws_model['boundary'] = []
# ------------------------------
# initialization for robot
# position of [x,y]

class RVO_controller():#满足切入RVO条件后启用该函数
    def __init__(self,robot,goal,v,p):
        self.total_time = 2
        self.t=0
        self.chose=0#判决是否转回pid
        self.robot = robot
        self.goal = goal
        self.v = v#此处v为x方向与y方向，需魔改
        self.p = p

    def RVO(self):
        if self.t * step >= total_time:
            self.chose = 1
            # 计算理想速度，如果不存在障碍物的话，机器人朝向工作台的理想情况为二者之间的连线，且速度为最大
        V_des , self.chose = calcu_desire_V(self.robot, self.goal, V_max, self.chose)
        # compute the optimal vel to avoid collision
        self.v = RVO_update(self.robot, V_des, self.v, ws_model)
        # update position
        for i in range(4):
            self.robot[i][0] += self.v[i][0] * step
            self.robot[i][1] += self.v[i][1] * step
        # ----------------------------------------
        # visualization
        # visualize_traj_dynamic(ws_model, Robot, V, workstation, time=t * step, name='data/snap%d.png' % int(t / 10))
        # print('t = {}\n'.format(t))
        v_list = []
        w_list = []
        for j in range(4):
            theta = round(arctan2(self.v[j][1], self.v[j][0]) / PI, 3)
            liner_v = sqrt(self.v[j][0] ** 2 + self.v[j][1] ** 2)
            v_list.append(liner_v)
            w_pi=(self.p[j]-theta)#弧度制配合
            w=w_pi*9
            if w_pi > 1/3:
                w = 3
            elif w_pi < -1/3:
                w = -3
            w_list.append(w*PI/3)
            """
            print(
                'Robot_{} :    X_velocity = {}     |       Y_velocity = {}     |       Liner_velocity = {}      |       Theta = {}PI\n'.format(
                    j, V[j][0],
                    V[j][1], liner_v,
                     theta))
            """
        self.t += 1
        #if self.chose == 1:
            #RVO_controller.end()
        #else:
        return v_list,w_list

    def end(self):
        """调回PID     self.robot, self.goal, self.v, self.p
            或重启PID进程
        """
import matplotlib.pyplot as plt
Robot = [[10*i,10*i] for i in range(4)]
V = [[0, 0] for i in range(len(Robot))]
P = [0 for i in range(len(Robot))]
workstation = [[50-10*i,50-10*i] for i in range(4)]
plt.figure(figsize=(8, 8))
for i in range(500):
    rvo=RVO_controller(Robot,workstation,V,P)
    plt.cla()
    plt.scatter([x[0] for x in Robot], [x[1] for x in Robot])
    plt.xlim(0, 50)
    plt.ylim(0, 50)
    plt.pause(0.001)
    #print(f'\rRobot coord: {Robot}', end='')
    v,w=rvo.RVO()
    print(f'{v} {w}')
"""
#原代码
Robot = [[10*i,10*i] for i in range(4)]#bot坐标输入
# velocity of [vx,vy]
V = [[0, 0] for i in range(len(Robot))]
# max Velocity : 6m/s
V_max = 6
# workstation of [x,y]
workstation = [[50-10*i,50-10*i] for i in range(4)]#目标工作台输入

# ------------------------------
# simulation setup
# total simulation time (s)
total_time = 15
# simulation step
step = 0.01

# ------------------------------
# simulation starts
t = 0
chose = 0
while t * step < total_time:
    # 计算理想速度，如果不存在障碍物的话，机器人朝向工作台的理想情况为二者之间的连线，且速度为最大
    V_des = calcu_desire_V(Robot, workstation, V_max)
    # compute the optimal vel to avoid collision
    V = RVO_update(Robot, V_des, V, ws_model)
    # update position
    for i in range(len(Robot)):
        Robot[i][0] += V[i][0] * step
        Robot[i][1] += V[i][1] * step
    # ----------------------------------------
    # visualization
    if t % 10 == 0:
        #visualize_traj_dynamic(ws_model, Robot, V, workstation, time=t * step, name='data/snap%d.png' % int(t / 10))
        print('t = {}\n'.format(t))
        for j in range(4):
            theta = str(round(arctan2(V[j][1], V[j][0]) / PI, 3))
            liner_v = sqrt(V[j][0] ** 2 + V[j][1] ** 2)
            print(
                'Robot_{} :    X_velocity = {}     |       Y_velocity = {}     |       Liner_velocity = {}      |       Theta = {}PI\n'.format(
                    j, V[j][0],
                    V[j][1], liner_v,
                    theta))
    t += 1
"""
