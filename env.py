import numpy as np
import pybullet as p
import pybullet_data
import math
from config import Config
from pybullet_utils import bullet_client
from scipy.spatial.transform import Rotation as R
import sys

class Env:
    def __init__(self, cfgFile):
        self.config = Config(cfgFile);
        seed = self.config.Get("Env.Seed")
        if (seed == 'random'):
            if sys.platform != 'win32':
                import posix
                self.seed = int.from_bytes(posix.urandom(4))
            else:
                import win32
                self.seed = int.from_bytes(win32.urandom(4))
        else:
            self.seed = int(seed)
        self.step_num = 0
        self.max_steps = self.config.GetAs("User.MaxStep", int)
        self.gui = self.config.GetAs("Env.GUI", bool)
        self.p = bullet_client.BulletClient(connection_mode=p.GUI if self.gui else p.DIRECT)
        self.p.setGravity(self.config.GetAs("Sim.Gravity.X", float), self.config.GetAs("Sim.Gravity.Y", float), self.config.GetAs("Sim.Gravity.Z", float))
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.targetEnabled = self.config.GetAs("Scene.Target.Enabled", bool)
        self.obstacleEnabled = self.config.GetAs("Scene.Obstacle.Enabled", bool)
        self.acceptable = self.config.GetAs("Scoring.AcceptableRange", float)
        self.decreasing = self.config.GetAs("Scoring.DecreasingRange", float)
        self.fastfail = self.config.GetAs("Scoring.FastFail", bool)
        self.fastwin = self.config.GetAs("Scoring.FastWin", bool)
        self.collisionPenalty = self.config.GetAs("Scoring.CollisionPenalty", float)
        self.random_velocity = np.random.uniform(self.config.GetAs("Scene.Target.Motion.Lower", float), self.config.GetAs("Scene.Target.Motion.Upper", float), 2)
        self.init_env()

    def init_env(self):
        np.random.seed(self.seed)  
        self.fr5 = self.p.loadURDF("fr5_description/urdf/fr5v6.urdf", useFixedBase=True, basePosition=[0, 0, 0],
                                   baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi]), flags=p.URDF_USE_SELF_COLLISION)
        self.table = self.p.loadURDF("table/table.urdf", basePosition=[0, 0.5, -0.63],
                                      baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi / 2]))
        collision_target_id = self.p.createCollisionShape(shapeType=p.GEOM_CYLINDER, radius=0.02, height=0.05)
        self.target = self.p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_target_id, basePosition=[0.5, 0.8, 2])
        if self.obstacleEnabled:
            collision_obstacle_id = self.p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=0.1)
            self.obstacle1 = self.p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_obstacle_id, basePosition=[0.5, 0.5, 2])
        self.reset()

    def reset(self):
        self.step_num = 0
        self.success_reward = 0
        self.terminated = False
        self.obstacle_contact = False
        neutral_angle = [-49.45849125928217, -57.601209583849, -138.394013961943, -164.0052115563118, -49.45849125928217, 0, 0, 0]
        neutral_angle = [x * math.pi / 180 for x in neutral_angle]
        self.p.setJointMotorControlArray(self.fr5, [1, 2, 3, 4, 5, 6, 8, 9], p.POSITION_CONTROL, targetPositions=neutral_angle)

        self.goalx = np.random.uniform(self.config.GetAs("Scene.Target.X.Lower", float), self.config.GetAs("Scene.Target.X.Upper", float), 1)
        self.goaly = np.random.uniform(self.config.GetAs("Scene.Target.Y.Lower", float), self.config.GetAs("Scene.Target.Y.Upper", float), 1)
        self.goalz = np.random.uniform(self.config.GetAs("Scene.Target.Z.Lower", float), self.config.GetAs("Scene.Target.Z.Upper", float), 1)
        self.target_position = [ self.goalx[0], self.goaly[0], self.goalz[0] ]
        self.p.resetBasePositionAndOrientation(self.target, self.target_position, [0, 0, 0, 1])
        
        if self.obstacleEnabled:
            self.obstacle1x = np.random.uniform(self.config.GetAs("Scene.Obstacle.X.Lower", float), self.config.GetAs("Scene.Obstacle.X.Upper", float), 1) + self.goalx[0]
            self.obstacle1y = np.random.uniform(self.config.GetAs("Scene.Obstacle.Y.Lower", float), self.config.GetAs("Scene.Obstacle.Y.Upper", float), 1)
            self.obstacle1z = np.random.uniform(self.config.GetAs("Scene.Obstacle.Z.Lower", float), self.config.GetAs("Scene.Obstacle.Z.Upper", float), 1)

            self.obstacle1_position = [ self.obstacle1x[0], self.obstacle1y[0], self.obstacle1z[0] ]
            self.p.resetBasePositionAndOrientation(self.obstacle1, self.obstacle1_position, [0, 0, 0, 1])
        for _ in range(100):
            self.p.stepSimulation()

        self.random_velocity = np.random.uniform(self.config.GetAs("Scene.Target.Motion.Lower", float), self.config.GetAs("Scene.Target.Motion.Upper", float), 2)
        self.p.resetBaseVelocity(self.target, linearVelocity=[self.random_velocity[0], 0, self.random_velocity[1]])

        return self.get_observation()

    def GetObstaclePosition(self):
        return np.array(self.p.getBasePositionAndOrientation(self.obstacle1)[0]) if self.obstacleEnabled else [ 0, -1, 0 ]
    
    def GetTargetPosition(self):
        return np.array(self.p.getBasePositionAndOrientation(self.target)[0]) if self.targetEnabled else self.target_position


    def get_observation(self):
        joint_angles = [self.p.getJointState(self.fr5, i)[0] * 180 / np.pi for i in range(1, 7)]
        obs_joint_angles = ((np.array(joint_angles, dtype=np.float32) / 180) + 1) / 2
        target_position = self.GetTargetPosition()
        obstacle1_position = self.GetObstaclePosition()
        self.observation = np.hstack((obs_joint_angles, target_position, obstacle1_position)).flatten().reshape(1, -1)
        return self.observation

    def step(self, action):
        if self.terminated:
            return self.reset_episode()
        
        self.step_num += 1
        joint_angles = [self.p.getJointState(self.fr5, i)[0] for i in range(1, 7)]
        action = np.clip(action, self.config.GetAs("User.Action.Lower", float), self.config.GetAs("User.Action.Upper", float))
        fr5_joint_angles = np.array(joint_angles) + (np.array(action[:6]) / 180 * np.pi)
        gripper = np.array([0, 0])
        angle_now = np.hstack([fr5_joint_angles, gripper])
        self.reward()
        self.p.setJointMotorControlArray(self.fr5, [1, 2, 3, 4, 5, 6, 8, 9], p.POSITION_CONTROL, targetPositions=angle_now)

        for _ in range(20):
            self.p.stepSimulation()

            
        target_position = self.p.getBasePositionAndOrientation(self.target)[0]
        if target_position[0] > 0.5 or target_position[0] < -0.5:
            self.p.resetBaseVelocity(self.target, linearVelocity=[-self.random_velocity[0], 0, self.random_velocity[1]])
        if target_position[2] > 0.5 or target_position[2] < 0.1:
            self.p.resetBaseVelocity(self.target, linearVelocity=[self.random_velocity[0], 0, -self.random_velocity[1]])

        return self.observation

    def GetPosDiff(self):
        gripper_pos = self.p.getLinkState(self.fr5, 6)[0]
        relative_position = np.array([0, 0, 0.15])
        rotation = R.from_quat(self.p.getLinkState(self.fr5, 7)[1])
        rotated_relative_position = rotation.apply(relative_position)
        gripper_centre_pos = np.array(gripper_pos) + rotated_relative_position
        target_position = self.GetTargetPosition()
#        target_position = np.array([self.goalx[0], self.goaly[0], self.goalz[0]])
        return gripper_centre_pos - target_position;

    def get_dis(self):
        return np.linalg.norm(self.GetPosDiff())

    def reward(self):
        # 获取与桌子和障碍物的接触点

        if self.obstacleEnabled:
            table_contact_points = self.p.getContactPoints(bodyA=self.fr5, bodyB=self.table)
            obstacle1_contact_points = self.p.getContactPoints(bodyA=self.fr5, bodyB=self.obstacle1)

            for contact_point in table_contact_points or obstacle1_contact_points:
                link_index = contact_point[3]
                if link_index not in [0, 1]:
                    self.obstacle_contact = True

        
        distance = self.get_dis()

        # 计算奖励
        if (self.fastwin and distance < self.acceptable and self.step_num <= self.max_steps) or \
           (self.fastfail and self.obstacle_contact) or \
           (self.step_num >= self.max_steps):
            if distance <= self.acceptable:
                self.success_reward = 100
            elif distance - self.acceptable <= self.decreasing:
                self.success_reward = 100 * (1 - ((distance - self.acceptable) / self.decreasing))
            else:
                self.success_reward = 0
            if self.obstacle_contact:
                self.success_reward *= self.collisionPenalty
                    
            self.terminated = True


    def reset_episode(self):
        self.reset()
        return self.step_num, self.get_dis()

    def close(self):
        self.p.disconnect()
