import numpy as np
from stable_baselines3 import PPO
from abc import ABC, abstractmethod

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
        
    def get_action(self, observation):
        # 输入观测值，返回动作
        action = np.random.uniform(-1, 1, 6)
        return action

# 示例：使用PPO预训练模型
class PPOAlgorithm(BaseAlgorithm):
    def __init__(self):
        self.model = PPO.load("model.zip", device="cpu")

    def get_action(self, observation):
        action, _ = self.model.predict(observation)
        return action
    

