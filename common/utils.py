import numpy as np
import random

def time_cost(frame_num):
    return (1 - np.sqrt(1 - (1 - frame_num / 9000) ** 2)) * 0.2 + 0.8

class Saver():
    def __init__(self) -> None:
        pass