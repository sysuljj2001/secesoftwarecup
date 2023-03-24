import numpy as np
from components.scheduler import Task

class T:
    def __init__(self) -> None:
        pass

    def tester(self, t):
        t.append(1)

class Test:
    def __init__(self) -> None:
        self.que = []
    
    def test(self):
        a = T()
        a.tester(self.que)
        print(self.que)

if __name__ == '__main__':
    a = np.array([1, 2, 3])
    b = np.tile(a, (2, 1))
    a = np.zeros((4, 10))
    a[1, 2] = 1
    print(a)