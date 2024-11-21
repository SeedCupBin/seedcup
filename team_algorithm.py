import numpy
# from stable_baselines3 import PPO
from abc import ABC, abstractmethod
import time;
import math;
if __package__ == None or __package__ == '':
    import utils;
else:
    from . import utils;

class BaseAlgorithm(ABC):
    @abstractmethod 
    def get_action(self, observation):
        pass

class MyCustomAlgorithm(BaseAlgorithm):
    def __init__(self):
        pass
        
    def get_action(self, observation):
        # print("Axle state: {}".format(observation[0][0:6]))
        while True:
            action = input().split()
            if (len(action) == 6):
                try:
                    return numpy.array([float(action[0]), float(action[1]), float(action[2]), float(action[3]), float(action[4]), float(action[5])])
                except: pass

if __name__ == '__main__':
    import test
    test.main(MyCustomAlgorithm())

# Segment 1 (base_link to j1_Link): No direct length given; relies on joint configuration.
# Segment 2 (j1_Link to j2_Link): 0.152 meters
# Segment 3 (j2_Link to j3_Link): 0.425 meters
# Segment 4 (j3_Link to j4_Link): 0.39501 meters
# Segment 5 (j4_Link to j5_Link): 0.1021 meters
# Segment 6 (j5_Link to j6_Link): 0.102 meters
# Segment 7 (j6_Link to hand_base_link): 0.12 meters