import numpy as np

a = [1, 2, 3, 4]
b = [2, 2, 4, 5]
for i, (x, y) in enumerate(zip(a, b)):
    print(i, x, y)