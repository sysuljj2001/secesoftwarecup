'''
调度器（Scheduler）模块：全局策略模块，用于调度多智能体的行为，最大化全局利益
'''

import numpy as np
import random
from typing import Any, List

# 2023/3/17 future: 定义一系列价值函数

# 2023/3/18 future: 定义q表调用方法

class Task():
    def __init__(self, target_id: int, action: int) -> None:
        ''': action: 0 Sell 1 Buy 2 Destory
        
        : target_id: -1 ~ target_num - 1
        '''
        self.target_id = target_id
        self.action = action

class TaskQueue():
    def __init__(self, length) -> None:
        self.task_queue = []
        self.len = length
    
    def __getitem__(self, key):
        if key < len(self.task_queue):
            return self.task_queue[key]
        return None

    def activate(self):
        if len(self.task_queue):
            task = self.task_queue[0]
            del self.task_queue[0]
        return task
    
    def add_event(self, event: Task):
        if len(self.task_queue) < self.len:
            self.task_queue.append(event)
    
    def get_event(self):
        if len(self.task_queue):
            return self.task_queue[0]
        return None
    
    def check_event(self, map_status):
        '''检查队首的任务是否可以执行
        '''
        pass

class Scheduler():
    def __init__(self, bot_num, metadata, queue_len) -> None:
        self.bot_tasks = {i + 1 : TaskQueue(queue_len) for i in range(bot_num)}
        self.engine = None       # 机器人决策引擎
        self.metadata = metadata

    def plan(self, map_status):
        '''规划机器人的决策
        '''
        pass

    def finish(self, bot_id):
        '''机器人完成任务
        '''
        self.bot_tasks[bot_id].activate()

    def feedback(self):
        '''返回上次 plan 后每个机器人的决策
        '''
        pass

# Q-learning 调度器
class QTableScheduler(Scheduler):
    def __init__(self, bot_num, metadata, queue_len, file_path: str) -> None:
        super().__init__(bot_num, metadata, queue_len)
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

    def feedback(self):
        pass

# 贪心调度器
class SimpleScheduler(Scheduler):
    def __init__(self, bot_num, metadata, queue_len) -> None:
        super().__init__(bot_num, metadata, queue_len)
    
    def plan(self, map_status):
        pass

    def feedback(self):
        pass