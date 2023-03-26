'''
调度器（Scheduler）模块：全局策略模块，用于调度多智能体的行为，最大化全局利益
'''

import numpy as np
import random
from typing import Any, List
from components.preprocess import DataLoader, TARGET_MAT, BUY_COST, SELL_REWARD, PROCESS_TIME, MAP_SIZE
import logging
import math


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
    
    def __eq__(self, __value) -> bool:
        return self.action == __value.action and self.target_id == __value.target_id


class TaskQueue():
    def __init__(self, length, bot_id) -> None:
        self.task_queue = []
        self.len = length
        self.bot_id = bot_id

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

    def check_event(self, map_status: DataLoader):
        '''检查队首的任务是否可以执行
        '''
        event = self.get_event()
        if event is None: return False
        table = map_status.tables[event.target_id]
        bot = map_status.bots[self.bot_id]
        #logging.info(f'checking event {event.target_id}, {event.action}, {self.bot_id}')
        if event.action == 1:
            # 是否可以买
            if bot['item_type'] == 0 and table['prod_status'] == 1:
                return True
        elif event.action == 0:
            # 是否可卖
            if bot['item_type'] in map_status.valid_mat(event.target_id) and bot['item_type'] != 0:
                return True
        elif event.action == 2:
            return True
        return False

# 2023/03/22 future: 贪心调度器测试，每次让机器人去最近的工作台做能做的事情
# 2023/03/22 feature: 简单调度器测试，每次让机器人去最近工作台，啥都不干


class Scheduler():
    def __init__(self, bot_num, metadata, queue_len, engine=None, finder=None) -> None:
        '''简单机器人调度器，输入地图状态和机器人状态，输出当前状态的决策（去哪个工作台干什么）
        '''
        self.bot_tasks = [TaskQueue(queue_len, i) for i in range(bot_num)]
        self.engine = engine       # 机器人决策引擎
        self.finder = finder       # 机器人寻路引擎
        self.metadata = metadata
        self.queue_len = queue_len

    def plan(self, bot_id, map_status: DataLoader):
        bot_x, bot_y = map_status.bots[bot_id]['coord']
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
        # 随机找一个工作台去
        for bot_id, bot in enumerate(map_status.bots):
            bot_x, bot_y = bot['coord']
            while True:
                ret = random.sample(map_status.tables, 1)[0]
                table_x, table_y = ret['coord']
                ang = map_status.bots[bot_id]['p']
                if np.sqrt((table_x - bot_x) ** 2 + (table_y - bot_y) ** 2) > 10:
                    path = self.finder.planning(
                        bot_x, bot_y, table_x, table_y)[1:]
                    event = Task(path, 2, ret['id'])
                    self.bot_tasks[bot_id].add_event(event)
                    break
    
    def check_valid(self, bot_id, map_status: DataLoader):
        return self.bot_tasks[bot_id].check_event(map_status)

    def check_finish(self, bot_id, map_status: DataLoader):
        '''检查机器人完成任务情况
        '''
        event = self.bot_tasks[bot_id].get_event()
        if event is None:
            return False
        action = event.action_list[str(event.action)]

        table = map_status.tables[event.target_id]
        bot = map_status.bots[bot_id]
        table_type = table['table_type']
        
        if action == 'destroy':
            return True
        # 到达任务点，检查任务是否可用
        if bot['at_id'] == table['id']:
            if action == 'buy' and table['prod_status'] == 1 and bot['item_type'] == 0:
                return True
            if action == 'sell' and bot['item_type'] not in table['mat_status'] \
               and bot['item_type'] in TARGET_MAT[table_type - 1] and bot['item_type'] != 0:
                return True
            return False
        else:
            return False

    def feedback(self, bot_id):
        '''返回上次 plan 后每个机器人的决策
        '''
        return self.bot_tasks[bot_id].get_event()
    
    def feedback_all(self):
        return [self.bot_tasks[bot_id].get_event() for bot_id in range(len(self.bot_tasks))]

    def activate(self, bot_id):
        return self.bot_tasks[bot_id].activate()

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
    def __init__(self, bot_num, metadata, queue_len, engine=None, finder=None) -> None:
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
        ret = self.finder.planning(
            bot['coord'][0], bot['coord'][1], gx, gy, table_id)
        # 按路径长度排序
        ret = sorted(ret, key=lambda x: len(x['path']))
        # 带了物品
        if bot['item_type'] != 0:
            flag = False
            for pair in ret:
                path, table_id = pair['path'], pair['table_id']
                if bot['at_id'] == table_id:
                    continue
                table_type = tables[table_id]['table_type']
                # 最近可卖
                if bot['item_type'] in map_status.valid_mat(table_id) and bot['item_type'] != 0:
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
                if bot['at_id'] == table_id:
                    continue
                if tables[table_id]['prod_status'] == 1 and bot['item_type'] == 0:
                    event = Task(path, 1, table_id)
                    self.bot_tasks[bot_id].add_event(event)
                    flag = True
                    break
            if flag == False:
                event = Task(ret[1]['path'], 1, ret[1]['table_id'])
                self.bot_tasks[bot_id].add_event(event)

    def glob_plan(self, map_status: DataLoader):
        for bot_id in range(len(map_status.bots)):
            self.plan(bot_id, map_status)

class State():
    def __init__(self, bot_id: int, map_status: DataLoader, paths: List) -> None:
        self.map_status = map_status
        self.bot_id = bot_id
        self.bot_item = map_status.bot_item(bot_id)
        self.bot_coord = map_status.bot_coord(bot_id)
        self.bot_at = map_status.bot_at(bot_id)
        self.paths = paths
        self.tasks = []

    def _get_path(self, target_id):
        for x in self.paths:
            if x['table_id'] == target_id:
                return x['path']

    def clear_tasks(self):
        self.tasks = []

    def sell(self, target_id):
        '''在当前地图状态下将{买入 target_id 处的物品}加入一个bot的任务队列
        '''
        ret = self._get_path(target_id)
        self.tasks.append(Task(ret, 0, target_id))

    def buy(self, target_id):
        '''在当前地图状态下将{在 target_id 处卖出物品}加入一个bot的任务队列
        '''
        ret = self._get_path(target_id)
        self.tasks.append(Task(ret, 1, target_id))

    def destroy(self):
        '''在当前地图状态下将{销毁自身携带的物品}加入一个bot的任务队列并置于首端
        '''
        bot_coord = self.map_status.bot_coord(self.bot_id)
        self.tasks.insert(0, Task([bot_coord], 2, -1))

class ValueMap():
    def __init__(self, map_status: DataLoader) -> None:
        '''势力图，都是二维矩阵，横机器人纵工作台，一个元素代表一个价值权重
        '''
        self.map_status = map_status

    # 2023/03/24 future: 优化代码，减少冗余
    def get_all(self, weight: List[float]):
        '''获取全局价值矩阵
        :param weight: 每个势力图的权重，维度为 5，势力图依次为
        :mat_value : 原材料势力图
        :prod_value : 产品势力图
        :left_time_value : 工作台剩余生产时间势力图
        :dis_value : 距离势力图
        :item_value : 机器人携带物品势力图
        '''
        func_list = [
            self.mat_value(),
            self.prod_value(),
            self.left_time_value(),
            self.dis_value(),
            self.item_value(),
        ]
        res = np.zeros((len(self.map_status.bots), 
                        self.map_status.table_num))
        #for i in range(5):
        #    logging.info(func_list[i])
        for i, ret in enumerate(func_list):
            res += ret * weight[i]
        return res

    def mat_value(self):
        """原材料势力图：需要原材料越少、原材料栏越空且能卖的物品价值越高的工作台效益越大
        """
        table_num = self.map_status.table_num
        # 每种工作台的卖出平均收益
        table_sell_mean = [np.mean([SELL_REWARD[mat - 1] for mat in recepie]) if len(recepie) else 0 for recepie in TARGET_MAT]
        ret = np.zeros(table_num)
        for i in range(table_num):
            # 对每个工作台计算效益
            table_type = self.map_status.table_type(i)
            # 原材料栏长度
            table_valid_mat = len(self.map_status.valid_mat(i))
            # 卖出平均收益
            table_sell_val = table_sell_mean[table_type - 1]
            table_need_mat = len(TARGET_MAT[table_type - 1])
            ret[i] = (table_valid_mat + table_need_mat) / 1500 * table_sell_val 
        return np.tile(ret, (len(self.map_status.bots), 1))

    def prod_value(self):
        '''产品势力图：物品价值越高的产品效益越大
        '''
        table_num = self.map_status.table_num
        ret = np.zeros(table_num)
        for i in range(table_num):
            table_type = self.map_status.table_type(i)
            prod_stat = self.map_status.prod_status(i)
            if table_type > 7:
                ret[i] = (np.sum(SELL_REWARD) - np.sum(BUY_COST)) / 1000
            else:
                ret[i] = np.exp(prod_stat) * (SELL_REWARD[table_type - 1] - BUY_COST[table_type - 1]) / 500
        return np.tile(ret, (len(self.map_status.bots), 1))

    def left_time_value(self):
        '''工作台剩余生产时间势力图：优先去时间短的，剩余时间越短效益越大
        '''
        table_num = self.map_status.table_num
        ret = np.zeros(table_num)
        for i in range(table_num):
            table_type = self.map_status.table_type(i)
            left_time = self.map_status.time_left(i)
            if table_type > 7:
                ret[i] += 100
            elif table_type < 4:
                ret[i] += 10
            else:
                ret[i] += np.log(PROCESS_TIME[table_type - 1] - left_time + 1)
        return np.tile(ret, (len(self.map_status.bots), 1))

    def dis_value(self):
        '''距离势力图：离机器人越近（不考虑避障）效益越大
        '''
        table_num = self.map_status.table_num
        bot_num = len(self.map_status.bots)
        ret = np.zeros((bot_num, table_num))
        for b_i in range(bot_num):
            bot_x, bot_y = self.map_status.bot_coord(b_i)
            for t_i in range(table_num):
                table_x, table_y = self.map_status.table_coord(t_i)
                ret[b_i, t_i] = MAP_SIZE * np.sqrt(2) - math.hypot(bot_x - table_x, bot_y - table_y)
        return ret

    def item_value(self):
        '''机器人携带物品势力图：机器人携带的物品价值越高、时间价值系数越大、碰撞价值系数越大效益越大
        '''
        bot_num = len(self.map_status.bots)
        ret = np.zeros((bot_num, 1))
        for i in range(bot_num):
            item_type = self.map_status.bot_item(i)
            time_coef, coll_coef = self.map_status.bot_coef(i)
            ret[i, 0] = SELL_REWARD[item_type - 1] * time_coef * coll_coef / 400
        return np.tile(ret, (1, self.map_status.table_num))

# 状态机+决策引擎+势力图+经济系统贪心
class StateMachineScheduler(Scheduler):
    def __init__(self, bot_num, metadata, queue_len, engine=None, finder=None) -> None:
        super().__init__(bot_num, metadata, queue_len, engine, finder)

    def plan(self, bot_id, map_status: DataLoader):
        '''进行基本的决策预处理，将数据交给决策引擎处理
        （1）最短路的获取
        （2）封装地图状态，便于决策引擎与调度器交互
        '''
        bot = map_status.bots[bot_id]
        tables = map_status.tables
        gx, gy, table_id = [], [], []
        for table in tables:
            gx.append(table['coord'][0])
            gy.append(table['coord'][1])
            table_id.append(table['id'])
        ret = self.finder.planning(bot['coord'][0], bot['coord'][1], gx, gy, table_id)
        ret = sorted(ret, key=lambda x: len(x['path']))
        state = State(bot_id, map_status, ret)
        res = self.engine.bot_plan(state)
        [self.bot_tasks[bot_id].add_event(task) for task in res.tasks]

    def glob_plan(self, map_status: DataLoader):
        '''对机器人任务队列进行重规划
        （1）在任务队列层解决任务冲突
        （2）尝试某些策略用于分配最优决策
        （3）计算势力图
        '''
        value_map = ValueMap(map_status)
        self.engine.glob_plan(map_status, value_map, self.bot_tasks)
        for i in range(len(map_status.bots)):
            if not self.bot_tasks[i].check_event(map_status):
                self.bot_tasks[i].activate()
                self.plan(i, map_status)
        for bot_id in range(len(map_status.bots)):
            if self.bot_tasks[bot_id].get_event() is None:
                self.plan(bot_id, map_status)