import numpy as np
from stable_baselines3 import PPO
from abc import ABC, abstractmethod
import time;
import math;

class BaseAlgorithm(ABC):
    @abstractmethod 
    def get_action(self, observation):
        """
        输入观测值，返回动作
        Args:
            observation: numpy array of shape (1, 12) 包含:
                - 6个关节角度 (归一化到[0,1])
                - 3个目标位置坐标
                - 3个障碍物位置坐标
        Returns:
            action: numpy array of shape (6,) 范围在[-1,1]之间
        """
        pass

class MyCustomAlgorithm(BaseAlgorithm):
    def __init__(self):
        # 自定义初始化
        pass
    def RoundNotify(self, observation):
        print("\033[94mtarget.X = {}".format(observation[0][6]));
        print("obstacle.X = {}".format(observation[0][9]));
        print("target.Rot ={}".format(math.atan(observation[0][7] / observation[0][6])));
        print("obstacle.Rot ={}\033[0m".format(math.atan(observation[0][10] / observation[0][9])));
        pass
    def GetAction(self, axleState, targetPos, obstaclePos):
        # Arguments are splitted here.
        action = [0, 0, 0, 0, 0, 0];
        rotH = math.atan(targetPos[1] / targetPos[0]) / math.pi - 0.25;
        while rotH < 0: rotH += 1
        action[0] = 1 if rotH - axleState[0] > 1e-2 else -1 if rotH - axleState[0] < -1e-2 else (rotH - axleState[0]) / 1e-2;
        # print("Target is at: {}".format(targetPos))
        # print("Target horizontal rotation: {}".format(rotH))
        action[1] = -1;
        action[2] = 1;
        action[4] = -0.8;
        return action;
        # return [0, 0, 0, 0, 0, 0]
        
    def get_action(self, observation):
        # print("Axle state: {}".format(observation[0][0:6]))
        # time.sleep(0.1); # Add a delay here to clearly see the actions.
        return self.GetAction(observation[0][0:6], observation[0][6:9], observation[0][9:12]);

# # 示例：使用PPO预训练模型
# class PPOAlgorithm(BaseAlgorithm):
#     def __init__(self):
#         self.model = PPO.load("model.zip", device="cpu")
#     def get_action(self, observation):
#         action, _ = self.model.predict(observation)
#         return action

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