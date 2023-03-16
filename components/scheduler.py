import numpy as np
import random
from typing import Any, List

# 2023/3/17 future: 定义一系列价值函数

class Event(object):
    def __init__(self, frame: int, bot_id: int, axis: List[int, int],
                 vel: float, pal: float, direction: float, at_target: int) -> None:
        self.frame = frame
        self.axis = axis
        self.vel = vel
        self.pal = pal
        self.direction = direction
        self.bot_id = bot_id
        self.at_target = at_target
        self.q_func = None
    
    def __call__(self, *args: Any, **kwds: Any) -> Any:
        pass

    def action():
        pass

class AbstractItem(Event):
    def __init__(self, frame: int, bot_id: int, axis: List[int, int], 
                 vel: float, pal: float, direction: float, value: float,
                 at_target: int, item_id: int, time_coef: float, col_coef: float) -> None:
        super().__init__(frame, bot_id, axis, vel, pal, direction, at_target)
        self.value = value
        self.item_id = item_id
        self.time_coef = time_coef
        self.col_coef = col_coef

class Cell(AbstractItem):
    def __init__(self, frame: int, bot_id: int, axis: List[int, int], 
                 vel: float, pal: float, direction: float, value: float, 
                 at_target: int, item_id: int, time_coef: float, col_coef: float) -> None:
        super().__init__(frame, bot_id, axis, vel, pal, direction, value, at_target, item_id, time_coef, col_coef)

    def action(self, status):
        nw_status = None # 对地图状态建模
        if self.at_target == -1:
            return 0, status
        else:
            command = f'sell {self.bot_id}'

class Buy(AbstractItem):
    pass

class Destroy(AbstractItem):
    pass

class Transport(Event):
    pass

class TaskQueue():
    pass

class Scheduler():
    pass