'''
决策引擎（engine）模块：针对不同的地图给予不同的特殊策略
'''

import numpy as np
import random
from components.scheduler import Task, TaskQueue, ValueMap, State
from components.preprocess import DataLoader, TARGET_MAT, BUY_COST, SELL_REWARD, PROCESS_TIME, MAP_SIZE
from typing import List, Callable
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
        :param replan_bot: 对单个机器人进行规划，等价于调用 bot_plan
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

    def glob_plan(self, map_status: DataLoader, value_map: ValueMap,
                  bot_tasks: List[TaskQueue]):
        # 先写个简单的冲突解决方案助助兴
        pass

class GeneralEngine(Engine):
    def __init__(self) -> None:
        super().__init__()
    
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
                    if tables[table_id]['prod_status'] == 1:
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
                # 如果地图上没有工作台可以卖出该物品，不买
                if tables[table_id]['prod_status'] == 1 and bot.bot_item == 0:
                    bot.buy(table_id)
                    flag = True
                    break
            if flag == False:
                if bot.bot_item == 0:
                    bot.buy(random.sample(bot.paths, 1)[0]['table_id'])
        return bot
    
    def glob_plan(self, map_status: DataLoader, value_map: ValueMap, bot_tasks: List[TaskQueue]):
        
        def pack_task(target_id, action):
            return Task([map_status.table_coord(table_id)], action, target_id)
        
        header_tasks = [bot_task.get_event() for bot_task in bot_tasks]
        # 如果要销毁，先进行全局决策

        # 如果要卖，

        # 根据势力图重规划
        weights = [0.3, 0.2, 0.3, 0.6, 0.5]
        force_map = value_map.get_all(weights)
        #logging.info(force_map)
        for bot_id in range(len(map_status.bots)):
            table_ids = np.argsort(force_map[bot_id, :])
            bot = map_status.bots[bot_id]
            for table_id in table_ids:
                table = map_status.tables[table_id]
                if header_tasks[bot_id] is None:
                    # 如果可以买
                    if bot['item_type'] == 0 and table['prod_status'] == 1:
                        event = pack_task(table_id, 1)
                        bot_tasks[bot_id].add_event(event)
                        break
                    # 如果可以卖
                    if bot['item_type'] in map_status.valid_mat(table_id) and bot['item_type'] != 0:
                        event = pack_task(table_id, 0)
                        bot_tasks[bot_id].add_event(event)
                        break
        # 解决冲突
        for i in range(len(header_tasks)):
            for j in range(i + 1, len(header_tasks)):
                if header_tasks[i] is None or header_tasks[j] is None: continue
                if header_tasks[i] == header_tasks[j]:
                    # 如果两任务相同
                    target_i = header_tasks[i].target_id
                    target_j = header_tasks[j].target_id
                    
                    # 选择效益较小的换到另一个同种类工作台
                    if force_map[i, target_i] < force_map[j, target_j]:
                        ad_target, ad_bot = target_i, i
                    else:
                        ad_target, ad_bot = target_j, j

                    # 根据势力图重新分配
                    action = header_tasks[i].action
                    bot = map_status.bots[ad_bot]
                    table = map_status.tables[ad_target]
                    table_ids = np.argsort(force_map[bot_id, :])

                    flag = False
                    for table_id in table_ids:
                        if table_id  == ad_target: continue
                        if action == 1:
                            # 如果可以买
                            if bot['item_type'] == 0 and table['prod_status'] == 1:
                                event = pack_task(table_id, 1)
                                bot_tasks[bot_id].add_event(event)
                                flag = True
                                logging.info('success')
                                break
                        if action == 2:
                            # 如果可以卖
                            if bot['item_type'] in map_status.valid_mat(table_id) and bot['item_type'] != 0:
                                event = pack_task(table_id, 0)
                                bot_tasks[bot_id].add_event(event)
                                logging.info('success')
                                flag = True
                                break
                    

class MapAEngine(Engine):
    def __init__(self) -> None:
        pass

    def bot_plan(self, bot: State):
        pass

    def glob_plan(self, map_status: DataLoader, value_map: ValueMap, bot_tasks: List[TaskQueue]):
        pass