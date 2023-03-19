import numpy as np
import random, re

def time_cost(frame_num):
    return (1 - np.sqrt(1 - (1 - frame_num / 9000) ** 2)) * 0.2 + 0.8

class Saver():
    def __init__(self, data: dict = None, path: str = None) -> None:
        self.data = data
        self.path = path
        self.template = '''class QTable():
    q_table = {
        #
    }'''

    def update(self, data: dict):
        self.data = data

    def save(self):
        text = ''
        for x in self.data.items():
            text += f'\'{x[0]}\': {list(x[1])},\n\t\t'
        ret = self.template.replace('#', text)
        with open(self.path, 'w') as file:
            file.write(ret)

if __name__ == '__main__':
    saver = Saver({'wow': 1, 'good': 2}, './testforsave.py')
    saver.save()