import numpy as np

def get_ang(rad):
    return 180 / np.pi * rad

print(get_ang(np.arctan(1 / 1)))