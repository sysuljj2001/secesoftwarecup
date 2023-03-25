#!/bin/bash
import sys
from components.finder import Dijkstra
from components.scheduler import Scheduler, SimpleScheduler, StateMachineScheduler
from components.preprocess import DataLoader
from components.controller import PID_Controller, RVO_Contorller
from components.engine import MapAEngine, AFuckingTestingEngine
import logging
import numpy as np

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
    controller = [PID_Controller() for _ in range(4)] #调用机器人控制类
    coll_controller = RVO_Contorller(0.45)
    finder = Dijkstra([0, 50], [0, 50], 2, 0.45)
    engines = [
        AFuckingTestingEngine(),
    ]
    scheduler = StateMachineScheduler(4, None, 2, finder=finder, engine=engines[0])
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
        # 2023/03/22 feature: 完成一个机器人的 PID 控制
        # 2023/03/22 future: 重构代码
        if dataloader.frame_id == 1:
            scheduler.glob_plan(dataloader)
            [x.refresh() for x in controller]
        
        coll_controller.update_bots(dataloader)
        # 是否启用 rvo
        is_rvo = coll_controller.glob_check(1)
        is_rvo = False
        if is_rvo:
            for i in range(4):
                if len(scheduler.bot_tasks[i].task_queue) <= 0:
                    scheduler.plan(i, dataloader)
            paths = [task.path for task in scheduler.feedback_all()]
            logging.info(paths)
            coll_controller.update_targets(dataloader, paths)
            rvo_res = coll_controller.handle(0.015)
            if rvo_res is None:
                coll_controller.refresh()
                is_rvo = False
        # 四个机器人分别执行任务和控制
        for i in range(4):
            if dataloader.frame_id % 100 == 0:
                scheduler.glob_plan(dataloader)
                [x.refresh() for x in controller]
            controller[i].update_bot(bot_infos[i])
            res = scheduler.check_finish(i, dataloader)
            if res and bot_status[i] == 3:
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

                s = dataloader.table_coord(event.target_id)
                logging.info(f'botid: {i}, targetid: {event.target_id}, targetpos: {s}, action: {event.action}')
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
                #if not scheduler.check_valid(i, dataloader):
                #    scheduler.activate(i)
                #    scheduler.plan(i, dataloader)
                
                event = scheduler.feedback(i)
                if event is None: continue
                path = [dataloader.table_coord(event.target_id)]  
                controller[i].update_path(path)
            # 判断使用哪一个控制器
            if not is_rvo:    
                res = controller[i].handle(0.015) # 当前帧所需控制指令
            else:
                res = rvo_res['v'][i], rvo_res['w'][i]
            if res is None: 
                controller[i].refresh()
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