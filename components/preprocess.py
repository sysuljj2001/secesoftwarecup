'''
预处理模块：用于读取输入数据并进行前处理，以供其它模块调配输入信息
'''

import numpy as np
import sys

MAP_SIZE = 50
TARGET_MAT = [[], [], [], [1, 2], [1, 3], [2, 3], [4, 5, 6], [7], [1, 2, 3, 4, 5, 6, 7]]
BUY_COST = [3000, 4400, 5800, 15400, 17200, 19200, 76000]
SELL_REWARD = [6000, 7600, 9200, 22500, 25000, 27500, 105000]
PROCESS_TIME = [50, 50, 50, 500, 500, 1000, 1, 1]

def get_coord(map_x: int, map_y: int):
    '''根据地图字符矩阵获取坐标
    '''
    return map_y / 4, 50 - map_x / 4

def bit_to_list(dec: int):
    bits = bin(dec)[:1:-1]
    ret = []
    [ret.append(i) if x == '1' else 0 for i, x in enumerate(bits)]
    return ret

class DataLoader():
    def __init__(self) -> None:
        '''数据加载器，存储所有读入数据以供其它模块调用
        '''
        self.tables = []
        self.obstacles = []
        self.table_num = 0
        self.bots = []
        
        self.frame_id = 0
        self.money = 20000

    def _read_all(self):
        '''读取数据，分行返回
        '''
        res = ''
        while True:
            ret = input()
            if ret == 'OK': break
            res += ret + '\n'
        return res.split('\n')[:-1]

    def init(self):
        '''初始化地图，地图的最左下角为原点，横x纵y
        '''
        ret = self._read_all()
        for i, line in enumerate(ret):
            for j, x in enumerate(line):
                if x == 'A': 
                    self.bots.append({
                            'id': len(self.bots), 
                            'coord': get_coord(i, j),
                            'at_id': -1,
                            'item_type': 0,
                            'time_coef': 0,
                            'coll_coef': 0,
                            'w': 0,
                            'v': [0, 0],
                            'p': 0,
                        })
                if '1' <= x <= '9':
                    self.tables.append({
                            'id': self.table_num, 
                            'coord': get_coord(i, j),
                            'table_type': int(x),
                            'remain_time': -1,
                            'mat_status': [],
                            'prod_status': 0,
                        })
                    self.table_num += 1
                    
        # 加入地图边界（虽然不知道有没有用）
        [(self.obstacles.append((0, i / 2)), 
         self.obstacles.append((i / 2, 0)), 
         self.obstacles.append((MAP_SIZE, i / 2)), 
         self.obstacles.append((i / 2, MAP_SIZE))) for i in range(0, MAP_SIZE << 1)]

    def read(self, first_line: str):
        '''读取帧数据
        '''
        # 读取第一行
        parts = first_line.split(' ')
        self.frame_id = int(parts[0])
        self.money = int(parts[1])

        # 处理剩余信息
        ret = self._read_all()
        self.table_num = int(ret[0])
        
        # 工作台信息
        for i in range(self.table_num):
            data = ret[i + 1].split(' ')
            self.tables[i]['table_type'] = int(data[0])
            self.tables[i]['coord'] = [float(data[1]), float(data[2])]
            self.tables[i]['remain_time'] = int(data[3])
            self.tables[i]['mat_status'] = bit_to_list(int(data[4]))
            self.tables[i]['prod_status'] = int(data[5])
        
        # 机器人信息
        for i in range(4):
            data = ret[self.table_num + i + 1].split(' ')
            self.bots[i]['at_id'] = int(data[0])
            self.bots[i]['item_type'] = int(data[1])
            self.bots[i]['time_coef'] = float(data[2])
            self.bots[i]['coll_coef'] = float(data[3])
            self.bots[i]['w'] = float(data[4])
            self.bots[i]['v'] = [float(data[5]), float(data[6])]
            self.bots[i]['p'] = float(data[7])
            self.bots[i]['coord'] = [float(data[8]), float(data[9])]
        
        sys.stdout.write('%d\n' % self.frame_id)
    
    def bot_at(self, bot_id: int):
        return self.bots[bot_id]['at_id']
    
    def bot_motion(self, bot_id: int):
        '''获取机器人运动状态(w, v, p)
        '''
        return self.bots[bot_id]['w'], self.bots[bot_id]['v'], self.bots[bot_id]['p']

    def bot_item(self, bot_id: int):
        return self.bots[bot_id]['item_type']
    
    def bot_coef(self, bot_id: int):
        '''获取机器人价值系数(time_coef, coll_coef)
        '''
        return self.bots[bot_id]['time_coef'], self.bots[bot_id]['coll_coef']

    def bot_coord(self, bot_id: int):
        return self.bots[bot_id]['coord']
        
    def table_type(self, table_id: int):
        return self.tables[table_id]['table_type']

    def table_coord(self, table_id: int):
        return self.tables[table_id]['coord']
        
    def valid_mat(self, table_id: int):
        '''获取工作台的当前可卖物品列表
        '''
        table_type = self.tables[table_id]['table_type']
        valid = TARGET_MAT[table_type - 1]
        mat_stat = self.tables[table_id]['mat_status']
        return list(set(valid) - set(mat_stat))
        
    def prod_status(self, table_id: int):
        '''获取工作台当前产品格状态
        '''
        return self.tables[table_id]['prod_status']
        
    def time_left(self, table_id: int):
        '''获取工作台生产剩余时间
        '''
        return self.tables[table_id]['remain_time']

if __name__ == '__main__':
    print(bit_to_list(48))

'''
1144 199346
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