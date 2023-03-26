#!/bin/bash
import sys
from components.finder import Dijkstra, SimpleFinder
from components.scheduler import Scheduler, SimpleScheduler, StateMachineScheduler
from components.preprocess import DataLoader
from components.controller import PID_Controller, RVO_Contorller, glob_check, avoid_collision
from components.engine import MapAEngine, AFuckingTestingEngine, GeneralEngine, MapBEngine, MapCEngine, MapDEngine
import logging
import numpy as np

logging.basicConfig(level=logging.DEBUG)
K = [43, 25, 47, 18]

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
    controller = [PID_Controller(kp=[4, 3.9], ki=[0.0007, 0.005], kd=[0.0005, 0.001]) for _ in range(4)] #调用机器人控制类
    #finder = Dijkstra([0, 50], [0, 50], 1.25, 0.45)
    finder = SimpleFinder()
    engines = [
        GeneralEngine(),
        MapAEngine(),
        MapBEngine(),
        MapCEngine(),
        MapDEngine(),
    ]
    index = K.index(dataloader.table_num)
    scheduler = StateMachineScheduler(4, None, 5, finder=finder, engine=engines[index + 1])
    # 机器人状态，1 执行任务 2 进行决策 3 移动
    bot_status = [3] * 4
    while True:
        # 读入一帧数据
        line = sys.stdin.readline()
        if not line:
            break
        dataloader.read(line)
        
        # 决策、控制
        bot_infos = dataloader.bots
        tables = dataloader.tables
        vel, ang_vel = np.zeros(4), np.zeros(4)
        # 2023/03/22 feature: 完成一个机器人的 PID 控制
        # 2023/03/22 future: 重构代码
        if dataloader.frame_id % 10 == 0:
            scheduler.glob_plan(dataloader)
    
        # 四个机器人分别执行任务和控制
        for i in range(4):
            #if dataloader.frame_id % 10 == 0:
            #    scheduler.glob_plan(dataloader)
            #    [x.refresh() for x in controller]
            controller[i].update_bot(bot_infos[i])
            ret = scheduler.check_finish(i, dataloader)
            if ret and bot_status[i] == 3:
                bot_status[i] = 1
                # 执行任务
                task = scheduler.activate(i)
                sys.stdout.write('%s %d\n' % (task.action_list[str(task.action)], i))
            elif bot_status[i] == 1:
                # 进行下一次规划，重置控制器
                bot_status[i] = 2
                controller[i].refresh()
                scheduler.plan(i, dataloader)
                scheduler.glob_plan(dataloader)
                event = scheduler.feedback(i)
                if event is None: continue

                s = dataloader.table_coord(event.target_id)
                logging.info(f'frame_id: {dataloader.frame_id}, botid: {i}, targetid: {event.target_id}, targetpos: {s}, targettype: {dataloader.table_type(event.target_id)}, action: {event.action}')
                if event.action == 2:
                    # 如果遭遇销毁指令，跳过该帧
                    bot_status[i] = 3
                    continue
                path = [dataloader.table_coord(event.target_id)]
                controller[i].update_path(path)
            elif bot_status[i] == 2 or bot_status[i] == 3:
                # 移动，并判断是否销毁，更新控制器信息
                bot_status[i] = 3
                if len(scheduler.bot_tasks[i].task_queue) <= 0:
                    scheduler.plan(i, dataloader)
                    scheduler.glob_plan(dataloader)
                
                event = scheduler.feedback(i)
                if event is None: continue
                path = [dataloader.table_coord(event.target_id)]  
                controller[i].update_path(path)
                res = controller[i].handle(0.015) # 当前帧所需控制指令

            if res is None: 
                controller[i].refresh()
                continue

            vel[i], ang_vel[i] = res[0], res[1]
        # 规避碰撞
        pos = [np.array(bot['coord']) for bot in bot_infos]
        ori = np.array([bot['p'] for bot in bot_infos])
        vel, ang_vel = avoid_collision(pos, vel, ang_vel, ori, 0.35, 5.5)
        for i in range(4):
            sys.stdout.write('forward %d %d\n' % (i, vel[i]))
            sys.stdout.write('rotate %d %f\n' % (i, ang_vel[i]))
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