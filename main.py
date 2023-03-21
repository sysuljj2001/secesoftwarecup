#!/bin/bash
import sys
from components import A_star, scheduler
from components.preprocess import DataLoader
from components.test import PID_Controller
from components.test import Robot

def read_util_ok():
    while input() != "OK":
        pass

def finish():
    sys.stdout.write('OK\n')
    sys.stdout.flush()


if __name__ == '__main__':
    dataloader = DataLoader()
    # 初始化地图
    dataloader.init()
    finish()
    while True:
        # 读入一帧数据
        line = sys.stdin.readline()
        if not line:
            break
        dataloader.read(line)
        
        # 决策、控制
        bot_infos = dataloader.bots
        # 2023/03/20 feature: 完成一个机器人的 PID 控制
        #for i in range(4)作各机器人判断
        needpath = A_find([bot_infos[0]['x'],bot_infos[0]['y']],[0,0])#假设点0,0，得出离散路径
        pid_controller = PID_Controller(bot_info=bot_infos[0], paths=needpath) #调用机器人控制类
        c1_1,c1_2 = pid_controller.handle()#当前帧所需控制指令
        print('0')
        print(f'forward {i} {c1_1}\n')
        print(f'rotate {i} {c1_2}\n')
        print('OK')
        '''
        line_speed, angle_speed = 3, 1.5
        for robot_id in range(4):
            sys.stdout.write('forward %d %d\n' % (robot_id, line_speed))
            sys.stdout.write('rotate %d %f\n' % (robot_id, angle_speed))
        '''
        finish()
