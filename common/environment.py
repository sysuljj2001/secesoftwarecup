import numpy as np
import threading
from typing import List
import random, re

TARGET_CLASS_NUM = 9
TIME_SPLIT = 10
BUY_COST = [3000, 4400, 5800, 15400, 17200, 19200, 76000]
SELL_REWARD = [6000, 7600, 9200, 22500, 25000, 27500, 10500]

# 2023/03/18 future: 更精确的状态定义，到达工作台时间分段计入状态

class Env():
    def __init__(self, n, end_frame) -> None:
        self.end_frame = end_frame
        self.frame = 0
        self.n = n

        self.bot_carry = 0
        self.close_targets = self._generate_targets()
        self.next_targets = self._generate_targets()

    def _generate_targets(self):
        ret = np.random.randint(1, TARGET_CLASS_NUM, self.n)
        return [
            {
                'type': ttype,
                'product': np.random.choice([1, 0]) if ttype < 8 else 0,
                'material': self._generate_meterial(ttype),
                'time': np.random.choice(range(TIME_SPLIT)),
            } for ttype in ret
        ]

    def _encode_status(self, bot_carry, close_targets):
        text = ''
        text += str(bot_carry) + '/'
        for x in close_targets:
            text += str(x['type']) + '/'
            text += str(x['product']) + '/'
            text += str(x['material']) + '/'
            text += str(x['time']) + '/'
        return text

    def decode_status(self, status: bytes):
        ret = re.split('/', status)[:-1]
        bot_carry = ret[0]
        close_targets = [
            {
                'type': int(x[0]),
                'product': int(x[1]),
                'material': [int(x) for x in re.findall(r'[1-9]', x[2])],
                'time': int(x[3]),
            }
            for x in [ret[i : i + 4] for i in range(1, len(ret), 4)]
        ]
        return bot_carry, close_targets

    def _generate_meterial(self, ttype: int):
        '''限制非法情况，减小状态空间
        '''
        ret = []
        k = 0
        if ttype == 4:
            k = 2
            ret.extend([1, 2])
        elif ttype == 5:
            k = 2
            ret.extend([1, 3])
        elif ttype == 6:
            k = 2
            ret.extend([2, 3])
        elif ttype == 7:
            k = 3
            ret.extend([4, 5, 6])
        elif ttype == 8:
            k = 1
            ret.extend([7])
        else:
            k = 6
            ret.extend(range(1, 7))
        return sorted(random.sample(ret, random.randint(0, k)))
    
    def draw(self):
        pass

    def get_terminal(self):
        '''结束一轮训练
        '''
        return self.frame == self.end_frame
    
    def get_observation(self):
        return self._encode_status(self.bot_carry, self.close_targets)

    def step(self, action):
        '''在环境中采取动作的状态转移
        '''
        self.frame += 1
        if action <= 2 ** self.n:
            idx = (action + 1) // 2
            '''
            if action == -1:
                # 不动转移（感觉没什么用）
                for i in range(len(self.close_targets)):
                    if self.close_targets[i]['time'] < TIME_SPLIT - 1:
                        self.close_targets[i]['time'] += 1
                    else:
                        if self.close_targets[i]['product'] == 0:
                            self.close_targets[i]['time'] = 0
                            self.close_targets[i]['product'] = 1
                        else:
                            self.close_targets[i]['time'] = TIME_SPLIT - 1'''
            if action == 0:
                # 销毁转移
                self.bot_carry = 0
                self.close_targets = self.next_targets
            elif action % 2 == 1:
                # 买入转移
                if self.close_targets[idx]['product'] and self.bot_carry == 0:
                    self.bot_carry = self.close_targets[idx]['type']
                self.close_targets = self.next_targets
            else:
                # 卖出转移
                if self.bot_carry != 0 and self.bot_carry not in self.close_targets[idx]['material']:
                    self.bot_carry = 0
                self.close_targets = self.next_targets
            self.next_targets = self._generate_targets()
    
    def _time_reward(self, t: int):
        return 2 * np.exp(-t / self.n) ** 0.5
    
    def _finish_reward(self, sc, ss, sl, sr, t):
        level = sc // 3
        # 总设计思想：工作台空闲时间尽可能少
        if t == 1:
            # 买入，工作台当前加工时间越长越好
            if ss == 1:
                return 0.3 * np.log(sl + 2) * level + len(sr) * 0.3
            else:
                return -0.2 * np.log(sl + 2) * level
        elif t == 0:
            # 卖出，工作台上有产品时，时间越短越好，无产品时，越长越好
            if ss == 1 and self.bot_carry not in sr:
                return np.exp(-sl / TIME_SPLIT) * level + len(sr) * 0.3
            elif ss == 0 and self.bot_carry not in sr:
                return 0.2 * np.exp(sl / TIME_SPLIT) * level + len(sr) * 0.3
            elif self.bot_carry in sr:
                return -0.2 * np.log(sl + 2) * level
        else:
            # 销毁，使得某个可能容纳该物品的工作台空闲变大
            return -0.2 * self.bot_carry // 3
        
    def _buy_reward(self, ss, sc):
        # 总思想：之后卖出的收益尽可能大
        if ss and self.bot_carry == 0:
            return 0.5 * (SELL_REWARD[sc - 1] - BUY_COST[sc - 1]) / 1000
        return 0
    
    def _sell_reward(self, sr, sc):
        if self.bot_carry not in sr and self.bot_carry != 0:
            return (SELL_REWARD[sc - 1] - BUY_COST[sc - 1]) / 1000
        return 0

    def get_target(self, action):
        '''在给定状态下采取动作的奖励
        '''
        if action <= 2 ** self.n:
            idx = (action + 1) // 2
            ret = self.close_targets[idx]
            sc = ret['type']
            ss = ret['product']
            sr = ret['material']
            sl = ret['time']
            if action == 0:
                score = self._time_reward(idx) + \
                        self._finish_reward(sc, ss, sl, sr, 2)
                return self._encode_status(0, self.next_targets), score
            elif action % 2 == 1:
                score = self._time_reward(idx) + \
                        self._finish_reward(sc, ss, sl, sr, 2) + \
                        self._buy_reward(ss, sc)
                if ss and self.bot_carry == 0:
                    return self._encode_status(sc, self.next_targets), score
                else:
                    return self._encode_status(self.bot_carry, self.next_targets), score
            else:
                score = self._time_reward(idx) + \
                        self._finish_reward(sc, ss, sl, sr, 2) + \
                        self._sell_reward(ss, sc)
                if self.bot_carry not in sr and self.bot_carry != 0:
                    return self._encode_status(0, self.next_targets), score
                else:
                    return self._encode_status(self.bot_carry, self), score

if __name__ == '__main__':
    test = Env(4, 9000)
    print(test._time_reward(1))