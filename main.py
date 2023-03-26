#!/bin/bash
import sys
from components.finder import Dijkstra, SimpleFinder
from components.scheduler import Scheduler, SimpleScheduler, StateMachineScheduler
from components.preprocess import DataLoader
from components.controller import PID_Controller, glob_check, avoid_collision
from components.engine import MapAEngine, GeneralEngine, MapBEngine, MapCEngine, MapDEngine
import logging
import numpy as np

logging.basicConfig(level=logging.DEBUG)
K = [43, 25, 50, 18]

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
    pid_param = [
        [[4, 8.5], [0.001, 0.1], [0.006, 0.5]],
        [[4, 8.5], [0.005, 0.05], [0.007, 0.4]],
        [[4.5, 9], [0.0005, 0.05], [0.008, 0.004]],
        [[4, 3.5], [0.0007, 0.01], [0.007, 0.001]],
    ]
    coll_param = [
        [0.1, 1.8], [0.1, 2], [0.1, 2.5], [0.1, 1.8]
    ]
    index = K.index(dataloader.table_num)

    controller = [PID_Controller(kp=pid_param[index][0], ki=pid_param[index][1], kd=pid_param[index][2]) for _ in range(4)] #调用机器人控制类
    finder = SimpleFinder()
    engines = [
        GeneralEngine(),
        MapAEngine(),
        MapBEngine(),
        MapCEngine(),
        MapDEngine(),
    ]
    
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
                if dataloader.frame_id >= 8800 and task.action == 1: continue
                if dataloader.frame_id >= 8600 and task.action == 1 and dataloader.tables[task.target_id]['table_type'] in [1, 2, 3]:
                    scheduler.glob_plan(dataloader)
                    continue
                    # 不买了
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
        vel, ang_vel = avoid_collision(pos, vel, ang_vel, ori, coll_param[index][0], coll_param[index][1])
        for i in range(4):
            sys.stdout.write('forward %d %d\n' % (i, vel[i]))
            sys.stdout.write('rotate %d %f\n' % (i, ang_vel[i]))
        finish()