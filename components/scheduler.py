import numpy as np
import random
from typing import Any, List

# 2023/3/17 future: 定义一系列价值函数

# 2023/3/18 future: 定义q表调用方法

class Event(object):
    def __init__(self, frame: int, bot_id: int, axis: List[int, int],
                 vel: float, pal: float, direction: float, target_id: int) -> None:
        self.frame = frame
        self.axis = axis
        self.vel = vel
        self.pal = pal
        self.direction = direction
        self.bot_id = bot_id
        self.target_id = target_id
        self.q_func = None
    
    def __call__(self, *args: Any, **kwds: Any) -> Any:
        pass

    def action():
        pass

class AbstractItem(Event):
    def __init__(self, frame: int, bot_id: int, axis: List[int, int], 
                 vel: float, pal: float, direction: float, value: float,
                 target_id: int, item_id: int, time_coef: float, col_coef: float) -> None:
        super().__init__(frame, bot_id, axis, vel, pal, direction, target_id)
        self.value = value
        self.item_id = item_id
        self.time_coef = time_coef
        self.col_coef = col_coef

class Sell(AbstractItem):
    def __init__(self, frame: int, bot_id: int, axis: List[int, int], 
                 vel: float, pal: float, direction: float, value: float, 
                 target_id: int, item_id: int, time_coef: float, col_coef: float) -> None:
        super().__init__(frame, bot_id, axis, vel, pal, direction, value, target_id, item_id, time_coef, col_coef)

    def action(self, status):
        nw_status = None # 对地图状态建模
        if self.target_id == -1:
            return '', status
        else:
            command = f'sell {self.bot_id}'
        return command, nw_status

class Buy(AbstractItem):
    pass

class Destroy(AbstractItem):
    pass

class Transport(Event):
    pass

class TaskQueue():
    def __init__(self, length) -> None:
        self.task_queue = []
        self.len = length
    
    def __getitem__(self, key):
        if key < len(self.task_queue):
            return self.task_queue[key]
        return None

    def activate(self):
        command = ''
        if len(self.task_queue):
            command, status = self.task_queue[0].action()
            del self.task_queue[0]
        return command, status
    
    def add_event(self, event: Event):
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

    def feedback(self):
        '''返回上次 plan 后每个机器人的决策
        '''
        pass

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

class SimpleScheduler(Scheduler):
    def __init__(self, bot_num, metadata, queue_len) -> None:
        super().__init__(bot_num, metadata, queue_len)
    
    def plan(self, map_status):
        pass

    def feedback(self):
        pass