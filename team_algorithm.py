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
        action[4] = -1;
        return action;
        
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