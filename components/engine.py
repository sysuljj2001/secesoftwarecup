'''
决策引擎（engine）模块：针对不同的地图给予不同的特殊策略
'''

import numpy as np
import random
from components.scheduler import Task, TaskQueue, ValueMap, State
from components.preprocess import DataLoader, TARGET_MAT, BUY_COST, SELL_REWARD, PROCESS_TIME, MAP_SIZE
from typing import List
import logging

class Engine():
    def __init__(self) -> None:
        pass

    def bot_plan(self, bot: State):
        '''单机器人决策入口
        :param bot: 抽象机器人类，用于执行该帧动作，可调用方法 buy(), sell(), destroy()
        :bot.paths 是一个路径列表，存储了 bot 到达所有工作台的最短路径，其中元素例 {'path': [(1, 1)], 'table_id': 1}
        :bot.tasks 是该机器人的任务队列，尽量不要动他，而使用 buy() 等方法增加可读性
        :bot.map_status 是该机器人在当前帧观测到的地图状态，与 preprocessor.Dataloader 定义一致

        :return bot: 同上，返回该机器人
        '''
        pass

    def glob_plan(self, map_status: DataLoader, value_map: ValueMap, bot_tasks: List[TaskQueue]):
        '''全局决策、重规划入口，直接对所有机器人任务队列进行任务重规划
        :param map_status: 该帧地图状态，与 preprocessor.Dataloader 定义一致
        :param value_map: 势力图，使用方法 value_map.get_all() 获取一个全局价值矩阵，详情见 ValueMap 类
        '''
        pass

class AFuckingTestingEngine(Engine):
    def __init__(self) -> None:
        pass

    def bot_plan(self, bot: State):
        tables = bot.map_status.tables
        if bot.bot_item != 0:
            flag = False
            for pair in bot.paths:
                path, table_id = pair['path'], pair['table_id']
                if bot.bot_at == table_id:
                    continue
                if bot.bot_item in bot.map_status.valid_mat(table_id) and bot.bot_item != 0:
                    bot.sell(table_id)
                    flag = True
                    if tables[table_id]['prod_status'] == 1 and bot.bot_item == 0:
                        bot.buy(table_id)
                    break
            if not flag:
                bot.destroy()
        else:
            flag = False
            for pair in bot.paths:
                path, table_id = pair['path'], pair['table_id']
                if bot.bot_at == table_id:
                    continue
                if tables[table_id]['prod_status'] == 1 and bot.bot_item == 0:
                    bot.buy(table_id)
                    flag = True
                    break
            if flag == False:
                #bot.buy(bot.paths[1]['table_id'])
                if bot.bot_item == 0:
                    bot.buy(random.sample(bot.paths, 1)[0]['table_id'])
        return bot

    def glob_plan(self, map_status: DataLoader, value_map: ValueMap, bot_tasks: List[TaskQueue]):
        # 先写个简单的冲突解决方案助助兴
        pass

class MapAEngine(Engine):
    def __init__(self) -> None:
        pass

    def bot_plan(self, bot: State):
        pass

    def glob_plan(self, map_status: DataLoader, value_map: ValueMap, bot_tasks: List[TaskQueue]):
        pass