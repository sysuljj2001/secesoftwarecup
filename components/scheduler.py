'''
调度器（Scheduler）模块：全局策略模块，用于调度多智能体的行为，最大化全局利益
'''

import numpy as np
import random
from typing import Any, List
from components.preprocess import DataLoader
import logging

TARGET_MAT = [[], [], [], [1, 2], [1, 3], [2, 3], [4, 5, 6], [], []]

# 2023/3/17 future: 定义一系列价值函数

# 2023/3/18 future: 定义q表调用方法

class Task():
    def __init__(self, path: List, action: int, target_id: int) -> None:
        '''
        :param action: 0 Sell 1 Buy 2 Destory
        :param path: 去一个工作台的路径
        :param target_id: 去哪个工作台
        '''
        self.path = path
        self.action = action
        self.target_id = target_id
        self.action_list = {
            '0': 'sell',
            '1': 'buy',
            '2': 'destroy'
        }

class TaskQueue():
    def __init__(self, length) -> None:
        self.task_queue = []
        self.len = length
    
    def __getitem__(self, key):
        if key < len(self.task_queue):
            return self.task_queue[key]
        return None

    def activate(self):
        task = None
        if len(self.task_queue):
            task = self.task_queue[0]
            del self.task_queue[0]
        return task
    
    def add_event(self, event: Task):
        if len(self.task_queue) < self.len:
            self.task_queue.append(event)
    
    def get_event(self) -> Task:
        if len(self.task_queue):
            return self.task_queue[0]
        return None
    
    def check_event(self, map_status):
        '''检查队首的任务是否可以执行
        '''
        pass

# 2023/03/22 future: 贪心调度器测试，每次让机器人去最近的工作台做能做的事情
# 2023/03/22 feature: 简单调度器测试，每次让机器人去最近工作台，啥都不干

class Scheduler():
    def __init__(self, bot_num, metadata, queue_len, engine = None, finder = None) -> None:
        '''简单机器人调度器，输入地图状态和机器人状态，输出当前状态的决策（去哪个工作台干什么）
        '''
        self.bot_tasks = [TaskQueue(queue_len) for i in range(bot_num)]
        self.engine = engine       # 机器人决策引擎
        self.finder = finder       # 机器人寻路引擎
        self.metadata = metadata

    def plan(self, bot_id, map_status: DataLoader):
        bot_x, bot_y = map_status.bots[bot_id]['coord']
        ang = map_status.bots[bot_id]['p']
        while True:
            ret = random.sample(map_status.tables, 1)[0]
            table_x, table_y = ret['coord']
            if np.sqrt((table_x - bot_x) ** 2 + (table_y - bot_y) ** 2) > 10:
                path = self.finder.planning(bot_x, bot_y, table_x, table_y)[1:]
                event = Task(path, 2, ret['id'])
                self.bot_tasks[bot_id].add_event(event)
                break

    def glob_plan(self, map_status: DataLoader):
        '''规划机器人的决策
        '''

        '''
        # 规划每个机器人到所有工作台的最短路径
        for bot_id, bot in enumerate(map_status.bots):
            robot_paths = []
            bot_x, bot_y = bot['coord']
            for id, table in enumerate(map_status.tables):
                table_x, table_y = table['coord']
                ret = self.finder.planning(bot_x, bot_y, table_x, table_y)[1:]
                ret.append(id)
                robot_paths.append(ret)
            # 寻找最短路径
            ret = sorted(robot_paths, key=lambda x : len(x))
            nearest_path, table_id = ret[0][:-1], ret[0][-1]

            # 策略：啥都不干
            event = Task(nearest_path, 2, table_id)
            self.bot_tasks[bot_id].add_event(event)'''
        # 随机找一个工作台去
        for bot_id, bot in enumerate(map_status.bots):
            bot_x, bot_y = bot['coord']
            while True:
                ret = random.sample(map_status.tables, 1)[0]
                table_x, table_y = ret['coord']
                ang = map_status.bots[bot_id]['p']
                if np.sqrt((table_x - bot_x) ** 2 + (table_y - bot_y) ** 2) > 10:
                    path = self.finder.planning(bot_x, bot_y, table_x, table_y)[1:]
                    event = Task(path, 2, ret['id'])
                    self.bot_tasks[bot_id].add_event(event)
                    break

    def check_finish(self, bot_id, map_status: DataLoader):
        '''检查机器人完成任务情况
        '''
        event = self.bot_tasks[bot_id].get_event()
        if event is None: return True

        table = map_status.tables[event.target_id]
        bot = map_status.bots[bot_id]
        table_x, table_y = table['coord']
        bot_x, bot_y = bot['coord']
        # 到达任务点，直接采取动作
        if np.sqrt((table_x - bot_x) ** 2 + (table_y - bot_y) ** 2) < 0.4:
            self.bot_tasks[bot_id].activate()
            return True
        else:
            return False

    def feedback(self):
        '''返回上次 plan 后每个机器人的决策
        '''
        return [x.get_event() for x in self.bot_tasks]

# Q-learning 调度器
class QTableScheduler(Scheduler):
    def __init__(self, bot_num, metadata, queue_len, engine, finder, file_path: str) -> None:
        super().__init__(bot_num, metadata, queue_len, engine, finder)
        self.bot_qtables, self.glob_qtables = self._load(file_path)

    def _load(self):
        '''从文件读取q表
        '''
        pass

    def convert_status(self, map_status, mtype):
        '''转换地图状态为q表状态
        '''
        if mtype == 'bot':
            pass
        elif mtype == 'glob':
            pass

    def plan(self, map_status):
        pass

    def feedback(self, map_status):
        pass

# 贪心调度器
class SimpleScheduler(Scheduler):
    def __init__(self, bot_num, metadata, queue_len, engine = None, finder = None) -> None:
        super().__init__(bot_num, metadata, queue_len, engine, finder)
    
    def plan(self, bot_id: int, map_status: DataLoader):
        '''简单贪心策略：找最近的收益最大的工作台执行动作，收益评判标准如下
        首先维护一个决策集合表示当前所有可执行的动作，包含如下内容
        （1）目标工作台可以买入不能卖出，且机器人未携带物品：
        （2）目标工作台可以买入不能卖出，且机器人携带物品：
        （3）不可买可卖。。。
        共8种，收益评判原则：没带物品优先去最近的买，带了物品优先去最近的卖
        '''
        # 计算机器人到所有工作台的最短路
        bot = map_status.bots[bot_id]
        tables = map_status.tables
        gx, gy, table_id = [], [], []
        for table in tables:
            gx.append(table['coord'][0])
            gy.append(table['coord'][1])
            table_id.append(table['id'])
        ret = self.finder.planning(bot['coord'][0], bot['coord'][1], gx, gy, table_id)
        # 按路径长度排序
        ret = sorted(ret, key=lambda x : len(x['path']))
        # 带了物品
        if bot['item_type'] != 0:
            flag = False
            for pair in ret:
                path, table_id = pair['path'], pair['table_id']
                if bot['at_id'] == table_id: continue
                table_type = tables[table_id]['table_type']
                # 最近可卖
                if bot['item_type'] not in tables[table_id]['mat_status'] \
                   and bot['item_type'] in TARGET_MAT[table_type - 1] and bot['item_type'] != 0:
                    event = Task(path, 0, table_id)
                    self.bot_tasks[bot_id].add_event(event)
                    flag = True
                    # 卖了直接买
                    if tables[table_id]['prod_status'] == 1:
                        event = Task([tables[table_id]['coord']], 1, table_id)
                        self.bot_tasks[bot_id].add_event(event)
                    break
            if not flag:
                # 买不到
                event = Task([bot['coord']], 2, table_id)
                self.bot_tasks[bot_id].add_event(event)
        # 没带物品
        else:
            flag = False
            for pair in ret:
                path, table_id = pair['path'], pair['table_id']
                if bot['at_id'] == table_id: continue
                if tables[table_id]['prod_status'] == 1 and bot['item_type'] == 0:
                    event = Task(path, 1, table_id)
                    self.bot_tasks[bot_id].add_event(event)
                    flag = True
                    break
            if flag == False:
                event = Task(ret[1]['path'], 1, ret[1]['table_id'])
                self.bot_tasks[bot_id].add_event(event)
    
    def check_finish(self, bot_id, map_status: DataLoader):
        event = self.bot_tasks[bot_id].get_event()
        if event is None: return False
        action = event.action_list[str(event.action)]

        table = map_status.tables[event.target_id]
        bot = map_status.bots[bot_id]
        table_type = table['table_type']
        # 到达任务点，检查任务是否可用
        if bot['at_id'] == table['id']:
            if action == 'buy' and table['prod_status'] == 1 and bot['item_type'] == 0:
                return True
            if action == 'sell' and bot['item_type'] not in table['mat_status'] and bot['item_type'] in TARGET_MAT[table_type - 1] and bot['item_type'] != 0:
                return True
            #self.bot_tasks[bot_id].activate()
            return False
        else:
            return False

    def glob_plan(self, map_status: DataLoader):
        for bot_id in range(len(map_status.bots)):
            self.plan(bot_id, map_status)

    def feedback(self, map_status):
        pass