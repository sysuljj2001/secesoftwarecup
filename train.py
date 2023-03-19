from common.environment import Env
from common.utils import Saver
from components.ql import AgentQL
import time

n = 2
end_frame = 100000
TIMES = 3000 # 训练轮数
SAVE_INTERVAL = 100

agent = AgentQL(n)
env = Env(n, end_frame)
saver = Saver(path='./qtable.py')
for epoch in range(TIMES):
    state = env.get_observation()
    while(True):
        done = env.get_terminal()
        env.draw()
        #time.sleep(0.2)
        if done:
            print(f'\n第{epoch + 1}次迭代，最终分数{env.money}，q 表大小 {len(agent.q_table)}，统计信息{env.static}')
            env.restart()
            if epoch % SAVE_INTERVAL == 0:
                saver.update(agent.q_table)
                saver.save()
            time.sleep(2)
            break
        action = agent.choose_action(state)
        state_after, score, pre_done = env.get_target(action)
        agent.learn(state, action, score, state_after, pre_done)
        env.step(action)
        state = state_after