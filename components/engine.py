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
                # 只有第一级物品可销毁
                if bot.bot_item <= 3:
                    bot.destroy()
                elif 3 < bot.bot_item <= 6:
                    s_tables = list(filter(lambda x : x['table_type'] == 7, tables))
                    bot.sell(random.sample(s_tables, 1)[0]['id'])
                else:
                    s_tables = list(filter(lambda x : x['table_type'] in [8, 9], tables))
                    bot.sell(random.sample(s_tables, 1)[0]['id'])
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
        # 获取场上可卖、可买物品列表
        sellable_item = []
        buyable_item = []
        bot_item_list = []
        for table_id in range(len(tables)):
            valid_list = bot.map_status.valid_mat(table_id)
            [sellable_item.append(x) for x in valid_list]
        [buyable_item.append(table['table_type']) if bot.map_status.prod_status(table['id']) else None for table in tables]
        [bot_item_list.append(bot['item_type']) if bot['item_type'] != 0 else 0 for bot in bot.map_status.bots]
        if bot.bot_item != 0:
            # 卖
            # 优先卖到原材料栏快满的
            # 如果有高级物品可以卖，优先卖高级物品
            ''' 这个暂时还是反向优化'''
            if len(set(sellable_item) & set([4, 5, 6, 7])) > 0:
                flag  = False
                s_tables = sorted(tables, key=lambda x : len(x['mat_status']))
                s_tables.reverse()
                for table in s_tables:
                    if bot.bot_item in bot.map_status.valid_mat(table['id']):
                        flag = True
                        if table['table_type'] == 9:
                            if bot.bot_item >= 7:
                                bot.sell(table['id'])
                                break
                        elif table['table_type'] >= 7:
                            bot.sell(table['id'])
                            break
                        elif table['table_type'] >= 4:
                            bot.sell(table['id'])
                            break
                        elif table['table_type'] != 9:
                            bot.sell(table['id'])
                            break
                if not flag:
                    # 到处都没得卖，如果
                    flag1 = False
                    if 3 < bot.bot_item <= 6:
                        s_tables = list(filter(lambda x : x['table_type'] == 7, tables))
                        if len(s_tables):
                            bot.sell(random.sample(s_tables, 1)[0]['id'])
                            flag1 = True
                    elif bot.bot_item > 6:
                        s_tables = list(filter(lambda x : x['table_type'] in [8, 9], tables))
                        if len(s_tables):
                            bot.sell(random.sample(s_tables, 1)[0]['id'])
                            flag1 = True
                    if not flag1:
                        bot.destroy()
            else:            
                flag = False
                for pair in bot.paths:
                    path, table_id = pair['path'], pair['table_id']
                    if bot.bot_item in bot.map_status.valid_mat(table_id) and bot.bot_item != 0:
                        if bot.map_status.table_type(table_id) == 9:
                            continue
                        bot.sell(table_id)
                        flag = True
                    if bot.bot_at == table_id:
                        if tables[table_id]['prod_status'] == 1 and bot.bot_item == 0:
                            # 如果买了没地方卖的，就换一个东西买
                            prod_type = tables[table_id]['table_type']
                            if prod_type not in sellable_item:
                                continue
                            bot.buy(table_id)
                if not flag:
                    # 只有第一级物品可销毁
                    bot.destroy()
        else:
            # 如果有高级物品可以买，优先买高级物品
            # 优先买缺的物品（可卖的高级物品）
            # 不买现在机器人拿的多的
            # 如果买到可能销毁，不买
            flag = False
            if len(set(buyable_item) & set([4, 5, 6, 7])) > 0:
                s_tables = sorted(tables, key=lambda x : x['table_type'])
                s_tables.reverse()
                for table in s_tables:
                    if bot_item_list.count(table['table_type']) >= 1:
                        continue
                    prod_type = table['table_type']
                    if prod_type not in sellable_item:
                        continue
                    if table['prod_status'] == 1 and table['table_type'] >= 7:
                        bot.buy(table['id'])
                        flag = True
                        break
                    elif table['prod_status'] == 1 and table['table_type'] >= 4:
                        bot.buy(table['id'])
                        flag = True
                        break
                    elif table['prod_status'] == 1 and table['table_type'] in sellable_item:
                        bot.buy(table['id'])
                        #logging.info('test1')
                        flag = True
                        break
            if not flag:
                flag1 = False
                for pair in bot.paths:
                    path, table_id = pair['path'], pair['table_id']
                    if bot_item_list.count(bot.map_status.tables[table_id]['table_type']) >= 2:
                        continue
                    if bot.bot_at == table_id:
                        continue
                    # 如果地图上没有工作台可以卖出该物品，不买
                    # 如果地图上有高级物品，不买低级物品
                    # 优先买能卖的地方多的物品
                    # 不买卖不出去的
                    if tables[table_id]['prod_status'] == 1 and bot.bot_item == 0:
                        prod_type = tables[table_id]['table_type']
                        if prod_type not in sellable_item:
                            continue
                        if sellable_item.count(prod_type) <= 2:
                            continue
                        bot.buy(table_id)
                        flag1 = True
                        break
                if not flag1:
                    # 到处都买不了，随便去一个快好的价值最高的工作台
                    s_tables = sorted(tables, key=lambda x : x['remain_time'])
                    for table in s_tables:
                        if table['table_type'] < 4: continue
                        #logging.info('test3')
                        bot.buy(table['id'])
                        break
        return bot
    
    def glob_plan(self, map_status: DataLoader, value_map: ValueMap, bot_tasks: List[TaskQueue]):
        
        def pack_task(target_id, action):
            return Task([map_status.table_coord(table_id)], action, target_id)
        tables = map_status.tables
        sellable_item = []
        buyable_item = []
        bot_item_list = []
        for table_id in range(len(tables)):
            valid_list = map_status.valid_mat(table_id)
            [sellable_item.append(x) for x in valid_list]
        [buyable_item.append(table['table_type']) if map_status.prod_status(table['id']) else None for table in tables]
        [bot_item_list.append(bot['item_type']) if bot['item_type'] != 0 else 0 for bot in map_status.bots]
        header_tasks = [bot_task.get_event() for bot_task in bot_tasks]
        # 如果要销毁，先进行全局决策

        # 如果要卖，先看是否有其它机器人要卖

        # 如果要买，先看是否有其他机器人要买

        # 优先买最紧缺的物品，填补物品

        # 根据势力图重规划
        weights = [1, 0.8, 0.6, 1.5, 0.5]
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
                        prod_type = table['table_type']
                        if prod_type not in sellable_item:
                            continue
                        #logging.info('test4')
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
                                prod_type = map_status.tables[table_id]['table_type']
                                # Weired Bug
                                if prod_type in sellable_item:
                                    event = pack_task(table_id, 1)
                                    bot_tasks[ad_bot].activate()
                                    bot_tasks[ad_bot].add_event(event)
                                    flag = True
                                    #logging.info(f'{table_id}, {ad_target}, {prod_type}, {sellable_item}')
                                    break
                        if action == 0:
                            # 如果可以卖
                            if bot['item_type'] in map_status.valid_mat(table_id) and bot['item_type'] != 0:
                                event = pack_task(table_id, 0)
                                bot_tasks[ad_bot].activate()
                                bot_tasks[ad_bot].add_event(event)
                                #logging.info('success')
                                flag = True
                                break
                    

class MapAEngine(Engine):
    def __init__(self) -> None:
        pass

    def bot_plan(self, bot: State):
        tables = bot.map_status.tables
        # 获取场上可卖、可买物品列表
        sellable_item = []
        buyable_item = []
        bot_item_list = []
        for table_id in range(len(tables)):
            valid_list = bot.map_status.valid_mat(table_id)
            [sellable_item.append(x) for x in valid_list]
        [buyable_item.append(table['table_type']) if bot.map_status.prod_status(table['id']) else None for table in tables]
        [bot_item_list.append(bot['item_type']) if bot['item_type'] != 0 else 0 for bot in bot.map_status.bots]
        desire_table_mat_list = [{'id':  table['id'],'status': table['mat_status']} for table \
                                   in bot.map_status.tables if table['table_type'] == 7] # 7 号工作台的原材料栏情况
        s_tables = sorted(tables, key=lambda x : len(x['mat_status']))
        s_tables.reverse()
        if bot.bot_item != 0:
            # 卖
            # 直接按顺序填 7 号工作台
            # 先卖高级物品，找到第一个材料不满的 7 号工作台的 id
            index = desire_table_mat_list[0]['id']
            for mat in desire_table_mat_list:
                if len(mat['status']) != 3 and tables[mat['id']]['remain_time'] == -1:
                    index = mat['id']
                    break
            # 判断7缺失的材料，优先令当前机器人去对应工作台卖
            desire_mat = bot.map_status.valid_mat(index)
            target_table_type = desire_mat[0]
            desire_raw = TARGET_MAT[target_table_type - 1]
            s_tables = sorted(tables, key=lambda x : len(x['mat_status']))
            if len(set(sellable_item) & set([4, 5, 6, 7])) > 0:
                # 地图 1 生产优先级 5, 6, 4，直接指定两个机器人优先卖到 5 的工作台
                flag  = False
                s_tables = sorted(tables, key=lambda x : len(x['mat_status']))
                s_tables.reverse()
                ss_tables = list(filter(lambda x : x['table_type'] == 6, s_tables))
                for table in ss_tables:
                    if bot.bot_item in bot.map_status.valid_mat(table['id']) and bot.bot_id in [0, 1]:
                        bot.sell(table['id'])
                        flag = True
                        break
                if not flag:
                    ss_tables = list(filter(lambda x : x['table_type'] == 5, s_tables))
                    for table in ss_tables:
                        if bot.bot_item in bot.map_status.valid_mat(table['id']) and bot.bot_id in [0, 1]:
                            bot.sell(table['id'])
                            flag = True
                            break
                # 如果机器人拿的材料是紧缺高级材料，直接去紧缺工作台卖
                if bot.bot_item in desire_mat and not flag:
                    bot.sell(index)
                    flag = True
                if not flag:
                    for table in s_tables:
                        if bot.bot_item in bot.map_status.valid_mat(table['id']) \
                           and table['table_type'] == target_table_type:
                            flag = True
                            if table['table_type'] == 9:
                                if bot.bot_item >= 7:
                                    bot.sell(table['id'])
                                    break
                            else:
                                bot.sell(table['id'])
                                break
        else:
            # 如果有高级物品可以买，优先买高级物品
            # 优先买缺的物品（可卖的高级物品）
            # 不买现在机器人拿的多的
            # 如果买到可能销毁，不买
            # 只有一个机器人负责买中间的
            flag = False
            if len(set(buyable_item) & set([4, 5, 6, 7])) > 0:
                # 地图 1 生产优先级 5, 6, 4
                # 无论低级物品是什么都直接买，指定一个机器人买中间
                # 优先买短缺高级物品，最优先买7
                index = desire_table_mat_list[0]['id']
                for mat in desire_table_mat_list:
                    if len(mat['status']) != 3:
                        index = mat['id']
                        break
                # 判断7缺失的材料，优先令当前机器人去对应工作台卖
                desire_mat = bot.map_status.valid_mat(index)
                s_tables = sorted(tables, key=lambda x : x['table_type'])
                s_tables.reverse()
                for table in s_tables:
                    if bot_item_list.count(table['table_type']) >= 1:
                        continue
                    prod_type = table['table_type']
                    if table['prod_status'] == 1 and table['table_type'] >= 7:
                        bot.buy(table['id'])
                        flag = True
                        break
                    if prod_type not in sellable_item or prod_type not in desire_mat:
                        continue
                    if table['prod_status'] == 1 and table['table_type'] >= 4:
                        bot.buy(table['id'])
                        flag = True
                        break
                    elif table['prod_status'] == 1 and table['table_type'] in sellable_item:
                        bot.buy(table['id'])
                        #logging.info('test1')
                        flag = True
                        break
            if not flag:
                # 先买最紧缺的基础材料
                # 找到第一个材料不满的 7 号工作台的 id
                index = desire_table_mat_list[0]['id']
                for mat in desire_table_mat_list:
                    if len(mat['status']) != 3:
                        index = mat['id']
                        break
                # 判断缺失的材料，令当前机器人去买原材料
                desire_mat = bot.map_status.valid_mat(index)
                desire_raw = TARGET_MAT[random.sample(desire_mat, 1)[0] - 1]
                target_table_type = random.sample(desire_raw, 1)[0]
                flag1 = False
                for pair in bot.paths:
                    path, table_id = pair['path'], pair['table_id']
                    prod_type = bot.map_status.tables[table_id]['table_type']
                    if bot_item_list.count(prod_type) >= 2:
                        continue
                    if bot.bot_at == table_id:
                        continue
                    # 如果地图上没有工作台可以卖出该物品，不买
                    # 如果地图上有高级物品，不买低级物品
                    # 优先买能卖的地方多的物品
                    # 不买卖不出去的
                    # 优先买最短缺的
                    if tables[table_id]['prod_status'] == 1 and bot.bot_item == 0 \
                       and prod_type == target_table_type:
                        prod_type = tables[table_id]['table_type']
                        if prod_type not in sellable_item:
                            continue
                        if sellable_item.count(prod_type) <= 1:
                            continue
                        bot.buy(table_id)
                        flag1 = True
                        break
                if not flag1:
                    # 到处都买不了，随便去一个快好的价值最高的工作台
                    s_tables = sorted(tables, key=lambda x : x['remain_time'])
                    for table in s_tables:
                        if table['table_type'] < 4: continue
                        #logging.info('test3')
                        bot.buy(table['id'])
                        break
        return bot
    
    def glob_plan(self, map_status: DataLoader, value_map: ValueMap, bot_tasks: List[TaskQueue]):
        
        def pack_task(target_id, action):
            return Task([map_status.table_coord(table_id)], action, target_id)
        tables = map_status.tables
        sellable_item = []
        buyable_item = []
        bot_item_list = []
        bot_target_list = []
        for table_id in range(len(tables)):
            valid_list = map_status.valid_mat(table_id)
            [sellable_item.append(x) for x in valid_list]
        [buyable_item.append(table['table_type']) if map_status.prod_status(table['id']) else None for table in tables]
        [bot_item_list.append(bot['item_type']) if bot['item_type'] != 0 else 0 for bot in map_status.bots]
        header_tasks = [bot_task.get_event() for bot_task in bot_tasks]
        bot_target_list = [task.target_id if task is not None else -1 for task in header_tasks]
        # 如果要销毁，先进行全局决策

        # 如果要卖，先看是否有其它机器人要卖

        # 如果要买，先看是否有其他机器人要买

        # 优先买最紧缺的物品，填补物品

        # 根据势力图重规划
        weights = [0.2, 1.2, 0.6, 0.1, 0.2]
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
                        prod_type = table['table_type']
                        if prod_type not in sellable_item:
                            continue
                        #logging.info('test4')
                        event = pack_task(table_id, 1)
                        bot_tasks[bot_id].add_event(event)
                        break
                    # 如果可以卖
                    if bot['item_type'] in map_status.valid_mat(table_id) and bot['item_type'] != 0:
                        event = pack_task(table_id, 0)
                        bot_tasks[bot_id].add_event(event)
                        break
        # 解决冲突，不允许多于两个机器人去同一个低级工作台
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
                                prod_type = map_status.tables[table_id]['table_type']
                                if bot_target_list.count(prod_type) <= 1 and prod_type <= 3: continue
                                if prod_type in sellable_item:
                                    event = pack_task(table_id, 1)
                                    bot_tasks[ad_bot].activate()
                                    bot_tasks[ad_bot].add_event(event)
                                    flag = True
                                    #logging.info(f'{table_id}, {ad_target}, {prod_type}, {sellable_item}')
                                    break
                        if action == 0:
                            # 如果可以卖
                            if bot['item_type'] in map_status.valid_mat(table_id) and bot['item_type'] != 0:
                                event = pack_task(table_id, 0)
                                bot_tasks[ad_bot].activate()
                                bot_tasks[ad_bot].add_event(event)
                                #logging.info('success')
                                flag = True
                                break

class MapBEngine(Engine):
    def __init__(self) -> None:
        pass

    def bot_plan(self, bot: State):
        tables = bot.map_status.tables
        # 获取场上可卖、可买物品列表
        sellable_item = []
        buyable_item = []
        bot_item_list = []
        for table_id in range(len(tables)):
            valid_list = bot.map_status.valid_mat(table_id)
            [sellable_item.append(x) for x in valid_list]
        [buyable_item.append(table['table_type']) if bot.map_status.prod_status(table['id']) else None for table in tables]
        [bot_item_list.append(bot['item_type']) if bot['item_type'] != 0 else 0 for bot in bot.map_status.bots]
        if bot.bot_item != 0:
            # 卖
            # 优先卖到原材料栏快满的
            # 如果有高级物品可以卖，优先卖高级物品
            ''' 这个暂时还是反向优化'''
            if len(set(sellable_item) & set([4, 5, 6, 7])) > 0:
                flag  = False
                s_tables = sorted(tables, key=lambda x : len(x['mat_status']))
                s_tables.reverse()
                for table in s_tables:
                    if bot.bot_item in bot.map_status.valid_mat(table['id']):
                        flag = True
                        if table['table_type'] == 9:
                            if bot.bot_item >= 7:
                                bot.sell(table['id'])
                                break
                        elif table['table_type'] >= 7:
                            bot.sell(table['id'])
                            break
                        elif table['table_type'] >= 4:
                            bot.sell(table['id'])
                            break
                        elif table['table_type'] != 9:
                            bot.sell(table['id'])
                            break
                if not flag:
                    # 到处都没得卖，如果
                    flag1 = False
                    if 3 < bot.bot_item <= 6:
                        s_tables = list(filter(lambda x : x['table_type'] == 7, tables))
                        if len(s_tables):
                            bot.sell(random.sample(s_tables, 1)[0]['id'])
                            flag1 = True
                    elif bot.bot_item > 6:
                        s_tables = list(filter(lambda x : x['table_type'] in [8, 9], tables))
                        if len(s_tables):
                            bot.sell(random.sample(s_tables, 1)[0]['id'])
                            flag1 = True
                    if not flag1:
                        bot.destroy()
            else:            
                flag = False
                for pair in bot.paths:
                    path, table_id = pair['path'], pair['table_id']
                    if bot.bot_item in bot.map_status.valid_mat(table_id) and bot.bot_item != 0:
                        if bot.map_status.table_type(table_id) == 9:
                            continue
                        bot.sell(table_id)
                        flag = True
                    if bot.bot_at == table_id:
                        if tables[table_id]['prod_status'] == 1 and bot.bot_item == 0:
                            # 如果买了没地方卖的，就换一个东西买
                            prod_type = tables[table_id]['table_type']
                            if prod_type not in sellable_item:
                                continue
                            bot.buy(table_id)
                if not flag:
                    # 只有第一级物品可销毁
                    bot.destroy()
        else:
            # 如果有高级物品可以买，优先买高级物品
            # 优先买缺的物品（可卖的高级物品）
            # 不买现在机器人拿的多的
            # 如果买到可能销毁，不买
            flag = False
            if len(set(buyable_item) & set([4, 5, 6, 7])) > 0:
                s_tables = sorted(tables, key=lambda x : x['table_type'])
                s_tables.reverse()
                for table in s_tables:
                    if bot_item_list.count(table['table_type']) >= 1:
                        continue
                    prod_type = table['table_type']
                    if prod_type not in sellable_item:
                        continue
                    if table['prod_status'] == 1 and table['table_type'] >= 7:
                        bot.buy(table['id'])
                        flag = True
                        break
                    elif table['prod_status'] == 1 and table['table_type'] >= 4:
                        bot.buy(table['id'])
                        flag = True
                        break
                    elif table['prod_status'] == 1 and table['table_type'] in sellable_item:
                        bot.buy(table['id'])
                        #logging.info('test1')
                        flag = True
                        break
            if not flag:
                flag1 = False
                for pair in bot.paths:
                    path, table_id = pair['path'], pair['table_id']
                    if bot_item_list.count(bot.map_status.tables[table_id]['table_type']) >= 2:
                        continue
                    if bot.bot_at == table_id:
                        continue
                    # 如果地图上没有工作台可以卖出该物品，不买
                    # 如果地图上有高级物品，不买低级物品
                    # 优先买能卖的地方多的物品
                    # 不买卖不出去的
                    if tables[table_id]['prod_status'] == 1 and bot.bot_item == 0:
                        prod_type = tables[table_id]['table_type']
                        if prod_type not in sellable_item:
                            continue
                        if sellable_item.count(prod_type) <= 2:
                            continue
                        bot.buy(table_id)
                        flag1 = True
                        break
                if not flag1:
                    # 到处都买不了，随便去一个快好的价值最高的工作台
                    s_tables = sorted(tables, key=lambda x : x['remain_time'])
                    for table in s_tables:
                        if table['table_type'] < 4: continue
                        #logging.info('test3')
                        bot.buy(table['id'])
                        break
        return bot
    
    def glob_plan(self, map_status: DataLoader, value_map: ValueMap, bot_tasks: List[TaskQueue]):
        
        def pack_task(target_id, action):
            return Task([map_status.table_coord(table_id)], action, target_id)
        tables = map_status.tables
        sellable_item = []
        buyable_item = []
        bot_item_list = []
        for table_id in range(len(tables)):
            valid_list = map_status.valid_mat(table_id)
            [sellable_item.append(x) for x in valid_list]
        [buyable_item.append(table['table_type']) if map_status.prod_status(table['id']) else None for table in tables]
        [bot_item_list.append(bot['item_type']) if bot['item_type'] != 0 else 0 for bot in map_status.bots]
        header_tasks = [bot_task.get_event() for bot_task in bot_tasks]
        # 如果要销毁，先进行全局决策

        # 如果要卖，先看是否有其它机器人要卖

        # 如果要买，先看是否有其他机器人要买

        # 优先买最紧缺的物品，填补物品

        # 根据势力图重规划
        weights = [0.9, 0.9, 0.5, 0.2, 0.5]
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
                        prod_type = table['table_type']
                        if prod_type not in sellable_item:
                            continue
                        #logging.info('test4')
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
                                prod_type = map_status.tables[table_id]['table_type']
                                # Weired Bug
                                if prod_type in sellable_item:
                                    event = pack_task(table_id, 1)
                                    bot_tasks[ad_bot].activate()
                                    bot_tasks[ad_bot].add_event(event)
                                    flag = True
                                    #logging.info(f'{table_id}, {ad_target}, {prod_type}, {sellable_item}')
                                    break
                        if action == 0:
                            # 如果可以卖
                            if bot['item_type'] in map_status.valid_mat(table_id) and bot['item_type'] != 0:
                                event = pack_task(table_id, 0)
                                bot_tasks[ad_bot].activate()
                                bot_tasks[ad_bot].add_event(event)
                                #logging.info('success')
                                flag = True
                                break

class MapCEngine(Engine):
    def __init__(self) -> None:
        self.forbidden_table = [
            0, 1, 2, 3, 8, 13, 14, 17, 23, 25, 26, 29, 30, 34, 44, 45, 46, 47, 48, 49
        ]

    def bot_plan(self, bot: State):
        tables = bot.map_status.tables
        if bot.bot_item != 0:
            flag = False
            for pair in bot.paths:
                path, table_id = pair['path'], pair['table_id']
                if bot.bot_at == table_id or table_id in self.forbidden_table:
                    continue
                if bot.bot_item in bot.map_status.valid_mat(table_id) and bot.bot_item != 0:
                    bot.sell(table_id)
                    flag = True
                    if tables[table_id]['prod_status'] == 1 and bot.bot_item == 0:
                        bot.buy(table_id)
                    break
            if not flag:
                # 只有第一级物品可销毁
                if bot.bot_item <= 3:
                    bot.destroy()
                elif 3 < bot.bot_item <= 6:
                    s_tables = list(filter(lambda x : x['table_type'] == 7, tables))
                    bot.sell(random.sample(s_tables, 1)[0]['id'])
                else:
                    s_tables = list(filter(lambda x : x['table_type'] in [8, 9], tables))
                    bot.sell(random.sample(s_tables, 1)[0]['id'])
        else:
            flag = False
            for pair in bot.paths:
                path, table_id = pair['path'], pair['table_id']
                if bot.bot_at == table_id or table_id in self.forbidden_table:
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
        
        def pack_task(target_id, action):
            return Task([map_status.table_coord(table_id)], action, target_id)
        tables = map_status.tables
        sellable_item = []
        buyable_item = []
        bot_item_list = []
        for table_id in range(len(tables)):
            valid_list = map_status.valid_mat(table_id)
            [sellable_item.append(x) for x in valid_list]
        [buyable_item.append(table['table_type']) if map_status.prod_status(table['id']) else None for table in tables]
        [bot_item_list.append(bot['item_type']) if bot['item_type'] != 0 else 0 for bot in map_status.bots]
        header_tasks = [bot_task.get_event() for bot_task in bot_tasks]
        # 如果要销毁，先进行全局决策

        # 如果要卖，先看是否有其它机器人要卖

        # 如果要买，先看是否有其他机器人要买

        # 优先买最紧缺的物品，填补物品

        # 根据势力图重规划
        weights = [0.4, 0.9, 0.5, 0.9, 0.6]
        force_map = value_map.get_all(weights)
        #logging.info(force_map)
        for bot_id in range(len(map_status.bots)):
            table_ids = np.argsort(force_map[bot_id, :])
            bot = map_status.bots[bot_id]
            for table_id in table_ids:
                if table_id in self.forbidden_table: continue
                table = map_status.tables[table_id]
                if header_tasks[bot_id] is None:
                    # 如果可以买
                    if bot['item_type'] == 0 and table['prod_status'] == 1:
                        prod_type = table['table_type']
                        if prod_type not in sellable_item:
                            continue
                        #logging.info('test4')
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
                                prod_type = map_status.tables[table_id]['table_type']
                                # Weired Bug
                                if prod_type in sellable_item:
                                    event = pack_task(table_id, 1)
                                    bot_tasks[ad_bot].activate()
                                    bot_tasks[ad_bot].add_event(event)
                                    flag = True
                                    #logging.info(f'{table_id}, {ad_target}, {prod_type}, {sellable_item}')
                                    break
                        if action == 0:
                            # 如果可以卖
                            if bot['item_type'] in map_status.valid_mat(table_id) and bot['item_type'] != 0:
                                event = pack_task(table_id, 0)
                                bot_tasks[ad_bot].activate()
                                bot_tasks[ad_bot].add_event(event)
                                #logging.info('success')
                                flag = True
                                break

class MapDEngine(Engine):
    def __init__(self) -> None:
        pass

    def bot_plan(self, bot: State):
        tables = bot.map_status.tables
        # 获取场上可卖、可买物品列表
        sellable_item = []
        buyable_item = []
        bot_item_list = []
        for table_id in range(len(tables)):
            valid_list = bot.map_status.valid_mat(table_id)
            [sellable_item.append(x) for x in valid_list]
        [buyable_item.append(table['table_type']) if bot.map_status.prod_status(table['id']) else None for table in tables]
        [bot_item_list.append(bot['item_type']) if bot['item_type'] != 0 else 0 for bot in bot.map_status.bots]
        if bot.bot_item != 0:
            # 卖
            # 优先卖到原材料栏快满的
            # 如果有高级物品可以卖，优先卖高级物品
            ''' 这个暂时还是反向优化'''
            if len(set(sellable_item) & set([4, 5, 6, 7])) > 0:
                flag  = False
                s_tables = sorted(tables, key=lambda x : len(x['mat_status']))
                s_tables.reverse()
                for table in s_tables:
                    if bot.bot_item in bot.map_status.valid_mat(table['id']):
                        flag = True
                        if table['table_type'] == 9:
                            if bot.bot_item >= 7:
                                bot.sell(table['id'])
                                break
                        elif table['table_type'] >= 7:
                            bot.sell(table['id'])
                            break
                        elif table['table_type'] >= 4:
                            bot.sell(table['id'])
                            break
                        elif table['table_type'] != 9:
                            bot.sell(table['id'])
                            break
                if not flag:
                    # 到处都没得卖，如果
                    flag1 = False
                    if 3 < bot.bot_item <= 6:
                        s_tables = list(filter(lambda x : x['table_type'] == 7, tables))
                        if len(s_tables):
                            bot.sell(random.sample(s_tables, 1)[0]['id'])
                            flag1 = True
                    elif bot.bot_item > 6:
                        s_tables = list(filter(lambda x : x['table_type'] in [8, 9], tables))
                        if len(s_tables):
                            bot.sell(random.sample(s_tables, 1)[0]['id'])
                            flag1 = True
                    if not flag1:
                        bot.destroy()
            else:            
                flag = False
                for pair in bot.paths:
                    path, table_id = pair['path'], pair['table_id']
                    if bot.bot_item in bot.map_status.valid_mat(table_id) and bot.bot_item != 0:
                        if bot.map_status.table_type(table_id) == 9:
                            continue
                        bot.sell(table_id)
                        flag = True
                    if bot.bot_at == table_id:
                        if tables[table_id]['prod_status'] == 1 and bot.bot_item == 0:
                            # 如果买了没地方卖的，就换一个东西买
                            prod_type = tables[table_id]['table_type']
                            if prod_type not in sellable_item:
                                continue
                            bot.buy(table_id)
                if not flag:
                    # 只有第一级物品可销毁
                    bot.destroy()
        else:
            # 如果有高级物品可以买，优先买高级物品
            # 优先买缺的物品（可卖的高级物品）
            # 不买现在机器人拿的多的
            # 如果买到可能销毁，不买
            flag = False
            if len(set(buyable_item) & set([4, 5, 6, 7])) > 0:
                s_tables = sorted(tables, key=lambda x : x['table_type'])
                s_tables.reverse()
                for table in s_tables:
                    if bot_item_list.count(table['table_type']) >= 1:
                        continue
                    prod_type = table['table_type']
                    if prod_type not in sellable_item:
                        continue
                    if table['prod_status'] == 1 and table['table_type'] >= 7:
                        bot.buy(table['id'])
                        flag = True
                        break
                    elif table['prod_status'] == 1 and table['table_type'] >= 4:
                        bot.buy(table['id'])
                        flag = True
                        break
                    elif table['prod_status'] == 1 and table['table_type'] in sellable_item:
                        bot.buy(table['id'])
                        #logging.info('test1')
                        flag = True
                        break
            if not flag:
                flag1 = False
                for pair in bot.paths:
                    path, table_id = pair['path'], pair['table_id']
                    if bot_item_list.count(bot.map_status.tables[table_id]['table_type']) >= 2:
                        continue
                    if bot.bot_at == table_id:
                        continue
                    # 如果地图上没有工作台可以卖出该物品，不买
                    # 如果地图上有高级物品，不买低级物品
                    # 优先买能卖的地方多的物品
                    # 不买卖不出去的
                    if tables[table_id]['prod_status'] == 1 and bot.bot_item == 0:
                        prod_type = tables[table_id]['table_type']
                        if prod_type not in sellable_item:
                            continue
                        if sellable_item.count(prod_type) <= 2:
                            continue
                        bot.buy(table_id)
                        flag1 = True
                        break
                if not flag1:
                    # 到处都买不了，随便去一个快好的价值最高的工作台
                    s_tables = sorted(tables, key=lambda x : x['remain_time'])
                    for table in s_tables:
                        if table['table_type'] < 4: continue
                        #logging.info('test3')
                        bot.buy(table['id'])
                        break
        return bot
    
    def glob_plan(self, map_status: DataLoader, value_map: ValueMap, bot_tasks: List[TaskQueue]):
        
        def pack_task(target_id, action):
            return Task([map_status.table_coord(table_id)], action, target_id)
        tables = map_status.tables
        sellable_item = []
        buyable_item = []
        bot_item_list = []
        for table_id in range(len(tables)):
            valid_list = map_status.valid_mat(table_id)
            [sellable_item.append(x) for x in valid_list]
        [buyable_item.append(table['table_type']) if map_status.prod_status(table['id']) else None for table in tables]
        [bot_item_list.append(bot['item_type']) if bot['item_type'] != 0 else 0 for bot in map_status.bots]
        header_tasks = [bot_task.get_event() for bot_task in bot_tasks]
        # 如果要销毁，先进行全局决策

        # 如果要卖，先看是否有其它机器人要卖

        # 如果要买，先看是否有其他机器人要买

        # 优先买最紧缺的物品，填补物品

        # 根据势力图重规划
        weights = [0.9, 0.5, 0.5, 1.2, 0.9]
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
                        prod_type = table['table_type']
                        if prod_type not in sellable_item:
                            continue
                        #logging.info('test4')
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
                                prod_type = map_status.tables[table_id]['table_type']
                                # Weired Bug
                                if prod_type in sellable_item:
                                    event = pack_task(table_id, 1)
                                    bot_tasks[ad_bot].activate()
                                    bot_tasks[ad_bot].add_event(event)
                                    flag = True
                                    #logging.info(f'{table_id}, {ad_target}, {prod_type}, {sellable_item}')
                                    break
                        if action == 0:
                            # 如果可以卖
                            if bot['item_type'] in map_status.valid_mat(table_id) and bot['item_type'] != 0:
                                event = pack_task(table_id, 0)
                                bot_tasks[ad_bot].activate()
                                bot_tasks[ad_bot].add_event(event)
                                #logging.info('success')
                                flag = True
                                break