import numpy
# from stable_baselines3 import PPO
from abc import ABC, abstractmethod
import time
import math

class Utils:
    def GetAngleFromPosition(pos):
        if pos[0] == 0:
            return math.pi / 2
        elif pos[0] < 0:
            return math.atan(pos[1] / pos[0]) + math.pi
        else:
            return math.atan(pos[1] / pos[0])

    def GetAxleRotationTransformation(current, target):
        return 1 if target - current > 1e-2 else -1 if target - current < -1e-2 else (target - current) / 1e-2

    def GetRectangularDistance(dis1, dis2):
        return math.sqrt(dis1 * dis1 + dis2 * dis2)

    def GetRectangularDistance(posArr):
        sq = 0
        for dis in posArr:
            sq += dis * dis
        return math.sqrt(sq)

    def GetTriangleAngle(c, a, b):
        # Get the angle opposite to edge C
        return math.acos((a * a + b * b - c * c) / 2 / a / b)

    def NormalizeAngle(angle):
        angle /= 2 * math.pi
        angle -= math.floor(angle)
        return angle

    def DenormalizeAngle(angle):
        return angle * 2 * math.pi

    def GetPoint2PlaneDistance(pos, angle):
        dist = Utils.GetRectangularDistance(pos)
        ang1 = Utils.GetAngleFromPosition(pos)
        ang2 = angle - ang1
        # print("\033[95mAngle = {} {} {}\033[0m".format(angle, ang1, ang2))
        return math.sin(ang2) * dist
    
    def GetStateHash(state):
        p = numpy.array([0, 0, 0, 0, 0, 0, 0, 0], dtype='uint8')
        for s in state:
            p ^= numpy.frombuffer(s.tobytes(), dtype='uint8')
        return int.from_bytes(p.tobytes(), byteorder = 'big')

class BaseAlgorithm(ABC):
    @abstractmethod 
    def get_action(self, observation):
        pass

class MyCustomAlgorithm(BaseAlgorithm):
    def __init__(self):
        self.arm2D1 = 0.425
        self.arm2D2 = 0.39501
        self.clawA = 0.29
        self.clawD = 0.36
        self.Debug = False
        self.Steps = 0
        self.Strategy = 0
        self.DistDir = 0
        self.DistAlt = 0
        self.DistSid = 0
        self.ArmStable = False
        self.StateHash = 0
        pass
    def DetermineStratrgy(self, targetPos, obstaclePos):
        self.Steps = 0
        self.Strategy = -1
        self.GetTargetAxleState(targetPos, obstaclePos)
        self.GetTargetAxleStateAlt(targetPos, obstaclePos)
        self.GetTargetAxleStateSide(targetPos, obstaclePos)
        # self.Strategy = 2 if self.DistSid > self.DistDir and self.DistSid > self.DistAlt else \
        #                 1 if self.DistAlt > self.DistDir and self.DistAlt > self.DistSid else \
        #                 0
        if targetPos[2] > obstaclePos[2] + 0.125 or self.DistDir < -0.138:
            self.Strategy = 0
        elif self.DistAlt > 0.2 and targetPos[1] > 0.85 or self.DistAlt > 0.3:
            self.Strategy = 1
        elif self.DistSid > 0.2:
            self.Strategy = 2
        else:
            self.Strategy = 1
        print("DistSid: {}".format(self.DistSid));
        print("DistAlt: {}".format(self.DistAlt));
        print("DistDir: {}".format(self.DistDir));
        print("Strategy: {}".format(self.Strategy));

    def RoundNotify(self, observation):
        # self.Debug = True
        # self.Debug = False
        # print("\033[94mDistDir: {}\033[0m".format(self.DistDir))
        # print("\033[94mDistAlt: {}\033[0m".format(self.DistAlt))
        # print("\033[94mtarget.X = {}".format(observation[0][6]))
        # print("obstacle.X = {}".format(observation[0][9]))
        # print("target.Rot ={}".format(math.atan(observation[0][7] / observation[0][6])))
        # print("obstacle.Rot ={}\033[0m".format(math.atan(observation[0][10] / observation[0][9])))
        pass
    def GetTargetAxleState(self, targetPos, obstaclePos):
        rotH = (Utils.GetAngleFromPosition(targetPos))
        if self.Strategy == -1: 
            self.DistDir = Utils.GetPoint2PlaneDistance(obstaclePos[0:2], rotH)
        else:
            targetAxleState2D = self.GetTargetAxleState2D([Utils.GetRectangularDistance(targetPos[0:2]) - 0.245, targetPos[2] - 0.05])
            rotH /= math.pi * 2
            return [rotH, targetAxleState2D[0], targetAxleState2D[1], targetAxleState2D[2], 0.17549, 0.5]

    def GetTargetAxleStateAlt(self, targetPos, obstaclePos):
        targetDistH = Utils.GetRectangularDistance(targetPos[0:2])
        rotH = (Utils.GetAngleFromPosition(targetPos) + math.asin(self.clawA / targetDistH))
        if self.Strategy == -1: 
            self.DistAlt = Utils.GetPoint2PlaneDistance(obstaclePos[0:2], rotH)
        else:
            rotH /= math.pi * 2
            if not self.ArmStable:
                rotH += 8 / 360
            distEq = math.sqrt(targetDistH * targetDistH - self.clawA * self.clawA)
            targetAxleState2D = self.GetTargetAxleState2D([distEq - 0.16, targetPos[2] - 0.05])
            return [rotH, targetAxleState2D[0], targetAxleState2D[1], targetAxleState2D[2], 0.4, 0.5]

    def GetTargetAxleStateSide(self, targetPos, obstaclePos):
        targetDistH = Utils.GetRectangularDistance(targetPos[0:2])
        rotH = (Utils.GetAngleFromPosition(targetPos) + math.asin(self.clawD / targetDistH))
        if self.Strategy == -1: 
            self.DistSid = Utils.GetPoint2PlaneDistance(obstaclePos[0:2], rotH)
            distEq = math.sqrt(targetDistH * targetDistH - self.clawD * self.clawD)
            dist = Utils.GetRectangularDistance([distEq, targetPos[2] - 0.05])
            if (dist > self.arm2D1 + self.arm2D2):
                self.DistSid = -1
        else:
            rotH /= math.pi * 2
            if not self.ArmStable:
                rotH += 4 / 360
            distEq = math.sqrt(targetDistH * targetDistH - self.clawD * self.clawD)
            targetAxleState2D = self.GetTargetAxleState2D([distEq, targetPos[2] - 0.05])
            return [rotH, targetAxleState2D[0], targetAxleState2D[1], targetAxleState2D[2], 0.5, 0.5]

    def GetTargetAxleState2D(self, targetPos2D):
        # if (self.Debug): print("pos:", targetPos2D)
        # print("Target Pos. 2D:", targetPos2D)
        dist = Utils.GetRectangularDistance(targetPos2D)
        if dist > self.arm2D1 + self.arm2D2:
            if (self.Debug): print("rots: N/A, N/A, N/A")
            return [0, 1, 0]
        rotV1 = Utils.GetAngleFromPosition(targetPos2D) + Utils.GetTriangleAngle(self.arm2D2, self.arm2D1, dist)
        rotV2 = Utils.GetTriangleAngle(dist, self.arm2D1, self.arm2D2)
        rotV3 = (1 * math.pi - rotV1 - rotV2) / math.pi / 2 #- 15 / 360 * math.pi * 2
        # rotV3 = -1 / 360 *  math.pi 
        # if (self.Debug): print("rot:", rotV1 / math.pi * 180, rotV2 / math.pi * 180, rotV3 / math.pi * 180)
        return [Utils.NormalizeAngle(rotV1), Utils.NormalizeAngle(rotV2), rotV3]

    def GetAction(self, axleState, targetPos, obstaclePos):
        curHash = Utils.GetStateHash(targetPos + obstaclePos)
        if curHash != self.StateHash:
            self.DetermineStratrgy(targetPos, obstaclePos)
            self.StateHash = curHash
        self.Steps += 1
        # Arguments are splitted here.
        action = [0, 0, 0, 0, 0, 0]
        if self.Strategy == 2:
            target = self.GetTargetAxleStateSide(targetPos, obstaclePos)
        elif self.Strategy == 1:
            target = self.GetTargetAxleStateAlt(targetPos, obstaclePos)
        else:
            target = self.GetTargetAxleState(targetPos, obstaclePos)
        # target = self.GetTargetAxleState([0, 0.9, 0.3], obstaclePos)
        self.ArmStable = True
        for i in range(6):
            action[i] = Utils.GetAxleRotationTransformation(axleState[i], target[i])
            if i != 0 and action[i] > 1e-1:
                self.ArmStable = False
        #    if (action[i] > 5e-2):
        # if final: print("Final! step =", self.moves)
        return action
        # return [0, 0, 0, 0, 0, 0]
        
    def get_action(self, observation):
        # print("Axle state: {}".format(observation[0][0:6]))
        # time.sleep(0.03) # Add a delay here to clearly see the actions.
        return numpy.array(self.GetAction(observation[0][0:6], observation[0][6:9], observation[0][9:12]))

if __name__ == '__main__':
    import test
    test.main(MyCustomAlgorithm())

# Segment 1 (base_link to j1_Link): No direct length given relies on joint configuration.
# Segment 2 (j1_Link to j2_Link): 0.152 meters
# Segment 3 (j2_Link to j3_Link): 0.425 meters
# Segment 4 (j3_Link to j4_Link): 0.39501 meters
# Segment 5 (j4_Link to j5_Link): 0.1021 meters
# Segment 6 (j5_Link to j6_Link): 0.102 meters
# Segment 7 (j6_Link to hand_base_link): 0.12 meters