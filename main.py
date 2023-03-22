#!/bin/bash
import sys
from components import A_star
from components.scheduler import Scheduler
from components.preprocess import DataLoader
from components.controller import PID_Controller
import logging

logging.basicConfig(level=logging.DEBUG)

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
    finder = A_star.Dijkstra([0, 50], [0, 50], 0.25, 0.45)
    scheduler = Scheduler(4, None, 5, finder=finder)
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

        if dataloader.frame_id == 1:
            scheduler.glob_plan(dataloader)
            [x.refresh() for x in pid_controller]

        # 以下操作统统在调度器中完成
        for i in range(4):
            if dataloader.frame_id % 100 == 1:
                scheduler.glob_plan(dataloader)
                [x.refresh() for x in pid_controller]

            if scheduler.check_finish(i, dataloader) or dataloader.frame_id == 1:
                pid_controller[i].refresh()
                scheduler.plan(i, dataloader)
                event = scheduler.bot_tasks[i].get_event()
                path = event.path
                pid_controller[i].update_path(path[::4])
                s = dataloader.tables[event.target_id]['coord']
                logging.info(f'botid: {i}, targetid: {event.target_id}, targetpos: {s}')
            
            pid_controller[i].update_bot(bot_infos[i])
            res = pid_controller[i].handle(0.015)#当前帧所需控制指令
            if res is None: 
                pid_controller[i].refresh()
                scheduler.plan(i, dataloader)
                continue
            sys.stdout.write('forward %d %d\n' % (i,res[0]))
            sys.stdout.write('rotate %d %f\n' % (i,res[1]))
        finish()


'''
1 199346
9
1 43.75 49.25 0 0 1
2 45.75 49.25 0 0 1
3 47.75 49.25 0 0 1
4 43.75 47.25 -1 0 0
5 45.75 47.25 168 0 0
6 47.75 47.25 -1 0 0
7 44.75 45.25 -1 0 0
8 46.75 45.25 -1 0 0
9 46.25 42.25 -1 0 0
5 3 0.9657950401 1 0 0 0 -0.3755806088 47.5760498 47.40252686
-1 0 0 0 0 0 0 -0.006108176429 43.75140762 48.23157501
-1 0 0 0 0 0 0 0 3.25 2.25
-1 0 0 0 0 0 0 0 45.75 1.75
OK
'''