from common.environment import Env
from components.ql import AgentQL
import time

n = 5
end_frame = 100000
TIMES = 30 # 训练轮数

agent = AgentQL(n)
env = Env(n, end_frame)
for epoch in range(TIMES):
    state = env.get_observation()
    while(True):
        done = env.get_terminal()
        if done:
            print(f'\n第{epoch + 1}次迭代')
            env.restart()
            time.sleep(2)
            break
        action = agent.choose_action(state)
        state_after, score, pre_done = env.get_target(action)
        agent.learn(state, action, score, state_after, pre_done)
        env.step(action)
        state = state_after