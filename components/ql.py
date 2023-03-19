import numpy as np
import random
from collections import defaultdict

class AgentQL():
    def __init__(self, n, lr=0.05, reward_decay=0.9, e_greedy=0.9) -> None:
        '''
        :param n: 一次动作可以选择到达的最优工作台数量
        :param lr: 学习率
        :param reward_decay: 期望系数
        :param e_greedy: 随机系数
        '''
        self.actions = range(2 * n + 1)
        self.lr = lr
        self.gamma = reward_decay
        self.epsilon = e_greedy
        self.q_table = defaultdict(lambda: np.array([0] * (2 * n + 1), dtype=np.float32))

    def choose_action(self, observation):
        '''在当前状态下选择一个动作执行
        '''
        action_list = self.q_table[observation]
        if np.random.uniform() < np.max(action_list):
            index = np.where(action_list - np.max(action_list) < 1e-2, 1, 0)
            ret = list(filter(lambda x : x != 0, [self.actions[i] if x != 0 else 0 for i, x in enumerate(index)]))
            action = np.random.choice(ret)
        else:
            action = np.random.choice(self.actions)
        return action

    def learn(self, ob_now, action, score, ob_af, done):
        '''更新 Q 表
        '''
        q_predict = self.q_table[ob_now][self.actions.index(action)]
        if done:
            q_target = score
        else:
            q_target = score + self.gamma * np.max(self.q_table[ob_af])
        self.q_table[ob_now][self.actions.index(action)] += self.lr * (q_target - q_predict)