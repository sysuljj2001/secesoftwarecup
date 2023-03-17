import os
from RVO import RVO_update, calcu_desire_V, PI
from vis import visualize_traj_dynamic
from numpy.random import random
from numpy import arctan2, sqrt

# ------------------------------
# define WorkStation model
ws_model = dict()
# robot radius
ws_model['robot_radius'] = 0.53
ws_model['circular_obstacles'] = []
ws_model['boundary'] = []
# ------------------------------
# initialization for robot
# position of [x,y]
Robot = [[random() * 50, random() * 50] for i in range(4)]
# velocity of [vx,vy]
V = [[0, 0] for i in range(len(Robot))]
# max Velocity : 6m/s
V_max = 6
# workstation of [x,y]
workstation = [[random() * 50, random() * 50] for i in range(4)]

# ------------------------------
# simulation setup
# total simulation time (s)
total_time = 15
# simulation step
step = 0.01

# ------------------------------
# simulation starts
t = 0
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
        visualize_traj_dynamic(ws_model, Robot, V, workstation, time=t * step, name='data/snap%d.png' % int(t / 10))
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

os.system('.\mkmovie.bat')
