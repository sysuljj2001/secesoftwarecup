#!/bin/bash
import sys
from components import A_star, scheduler
from components.preprocess import DataLoader
from components.controller import PID_Controller

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
    pid_controller = [PID_Controller() for _ in range(4)] #调用机器人控制类
    finder = A_star.Dijkstra([0, 50], [0, 50], 2, 0.45)
    paths = []
    while True:
        # 读入一帧数据
        line = sys.stdin.readline()
        if not line:
            break
        dataloader.read(line)
        
        # 决策、控制
        bot_infos = dataloader.bots
        # 2023/03/22 feature: 完成一个机器人的 PID 控制
        # 2023/03/22 future: 重构代码

        # 以下操作统统在调度器中完成
        for i in range(4):
            if dataloader.frame_id % 100 == 1:
                paths = finder.planning(bot_infos[i]['coord'][0], bot_infos[i]['coord'][1], 49, 1)
                pid_controller[i].refresh()
            pid_controller[i].update_bot(bot_infos[i])
            pid_controller[i].update_paths(paths[1::5])
            if len(paths) > 5: paths = paths[1:]
            res = pid_controller[i].handle(0.1)#当前帧所需控制指令
            if res is None: continue
            sys.stdout.write('forward %d %d\n' % (i,res[0]))
            sys.stdout.write('rotate %d %f\n' % (i,res[1]))
        finish()
