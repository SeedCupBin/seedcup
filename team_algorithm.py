import numpy
# from stable_baselines3 import PPO
from abc import ABC, abstractmethod
import time
import math

class Params:
    Arm2D1 = 0.425
    Arm2D2 = 0.39501
class Pos2:
    @property
    def Zero():
        return Pos2([0, 0])
    def __init__(self, arr):
        self.X = arr[0]
        self.Y = arr[1]
    def ToNpArray(self) -> numpy.ndarray:
        return numpy.array([self.X, self.Y])

class Pos3:
    def __init__(self, arr):
        self.X = arr[0]
        self.Y = arr[1]
        self.Z = arr[2]
    @property
    def XY(self) -> Pos2:
        return Pos2([self.X, self.Y])
    @property
    def XZ(self) -> Pos2:
        return Pos2([self.X, self.Z])
    @property
    def YZ(self) -> Pos2:
        return Pos2([self.Y, self.Z])
    def ToNpArray(self) -> numpy.ndarray:
        return numpy.array([self.X, self.Y, self.Z])

class Utils:
    def GetAngleFromPosition(pos : Pos2):
        if pos.X == 0:
            return math.pi / 2
        elif pos.X < 0:
            return math.atan(pos.Y / pos.X) + math.pi
        else:
            return math.atan(pos.Y / pos.X)

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

    def GetPoint2PlaneDistance(pos : Pos2, angle):
        dist = Utils.GetRectangularDistance(pos.ToNpArray())
        ang1 = Utils.GetAngleFromPosition(pos.ToNpArray())
        ang2 = angle - ang1
        # print("\033[95mAngle = {} {} {}\033[0m".format(angle, ang1, ang2))
        return math.sin(ang2) * dist

    def GetStateHash(state):
        p = numpy.array([0, 0, 0, 0, 0, 0, 0, 0], dtype='uint8')
        for s in state:
            p ^= numpy.frombuffer(s.tobytes(), dtype='uint8')
        return int.from_bytes(p.tobytes(), byteorder = 'big')

    def GetTargetAxleState2D(self, targetPos2D : Pos2):
        dist = Utils.GetRectangularDistance(targetPos2D.ToNpArray())
        if dist > Params.Arm2D1 + Params.Arm2D2:
            return [0, 1, 0]
        rotV1 = Utils.GetAngleFromPosition(targetPos2D) + Utils.GetTriangleAngle(self.arm2D2, self.arm2D1, dist)
        rotV2 = Utils.GetTriangleAngle(dist, Params.Arm2D1, Params.Arm2D2)
        rotV3 = (1 * math.pi - rotV1 - rotV2) / math.pi / 2
        return [Utils.NormalizeAngle(rotV1), Utils.NormalizeAngle(rotV2), rotV3]

class Strategy(ABC):
    @abstractmethod
    def __init__(self, algo):
        self.Algo = algo
    @abstractmethod
    def GetTargetAxleRotation(self, targetPos : Pos3, obstaclePos : Pos3): pass
    @abstractmethod
    def GetArm2ObstacleDistance(self, targetPos : Pos3, obstaclePos : Pos3): pass

class StrategyDirect(Strategy):
    def __init__(self, algo):
        super.__init__(algo)

    def GetArm2ObstacleDistance(self, targetPos : Pos3, obstaclePos : Pos3):
        rotH = (Utils.GetAngleFromPosition(targetPos))
        return Utils.GetPoint2PlaneDistance(obstaclePos.XY, rotH)

    def GetTargetAxleRotation(self, targetPos : Pos3, obstaclePos : Pos3):
        rotH = (Utils.GetAngleFromPosition(targetPos))
        targetDistH = Utils.GetRectangularDistance(targetPos.XY.ToNpArray())
        targetAxleState2D = Utils.GetTargetAxleState2D(Pos2([targetDistH - 0.245, targetPos.Z - 0.05]))
        rotH /= math.pi * 2
        return [rotH, targetAxleState2D[0], targetAxleState2D[1], targetAxleState2D[2], 0.17549, 0.5]

class StrategyAlternate(Strategy):
    def __init__(self, algo):
        super.__init__(algo)
        self.clawA = 0.29

    def GetArm2ObstacleDistance(self, targetPos : Pos3, obstaclePos : Pos3):
        targetDistH = Utils.GetRectangularDistance(targetPos.XY.ToNpArray())
        rotH = (Utils.GetAngleFromPosition(targetPos) + math.asin(self.clawA / targetDistH))
        return Utils.GetPoint2PlaneDistance(obstaclePos.XY, rotH)

    def GetTargetAxleRotation(self, targetPos : Pos3, obstaclePos : Pos3):
        targetDistH = Utils.GetRectangularDistance(targetPos.XY.ToNpArray())
        rotH = (Utils.GetAngleFromPosition(targetPos) + math.asin(self.clawA / targetDistH))
        rotH /= math.pi * 2
        if not self.Algo.armStable:
            rotH += 4 / 360
        distEq = math.sqrt(targetDistH * targetDistH - self.clawA * self.clawA)
        targetAxleState2D = Utils.GetTargetAxleState2D(Pos2([distEq - 0.16, targetPos.Z - 0.05]))
        return [rotH, targetAxleState2D[0], targetAxleState2D[1], targetAxleState2D[2], 0.4, 0.5]

class StrategySide(Strategy):
    def __init__(self, algo):
        super.__init__(algo)
        self.clawD = 0.36

    def GetArm2ObstacleDistance(self, targetPos : Pos3, obstaclePos : Pos3):
        targetDistH = Utils.GetRectangularDistance(targetPos.XY.ToNpArray())
        rotH = (Utils.GetAngleFromPosition(targetPos) + math.asin(self.clawD / targetDistH))
        distSid = Utils.GetPoint2PlaneDistance(obstaclePos.XY, rotH)
        distEq = math.sqrt(targetDistH * targetDistH - self.clawD * self.clawD)
        dist = Utils.GetRectangularDistance([distEq, targetPos.Z - 0.05])
        if (dist > Params.Arm2D1 + Params.Arm2D2):
            return -1
        return distSid

    def GetTargetAxleRotation(self, targetPos : Pos3, obstaclePos : Pos3):
        targetDistH = Utils.GetRectangularDistance(targetPos.XY.ToNpArray)
        rotH = (Utils.GetAngleFromPosition(targetPos) + math.asin(self.clawD / targetDistH))
        rotH /= math.pi * 2
        if not self.Algo.armStable:
            rotH += 4 / 360
        distEq = math.sqrt(targetDistH * targetDistH - self.clawD * self.clawD)
        targetAxleState2D = Utils.GetTargetAxleState2D(Pos2([distEq, targetPos.Z - 0.05]))
        return [rotH, targetAxleState2D[0], targetAxleState2D[1], targetAxleState2D[2], 0.5, 0.5]

class BaseAlgorithm(ABC):
    @abstractmethod 
    def get_action(self, observation):
        pass

class MyCustomAlgorithm(BaseAlgorithm):
    def __init__(self):
        self.Debug = False
        self.Steps = 0
        self.SelectedStrategy = 0
        self.Strategies = []
        self.TargetRot = [0, 0, 0, 0, 0, 0]
        self.DistDir = 0
        self.DistAlt = 0
        self.DistSid = 0
        self.ArmStable = False
        self.StateHash = 0
        self.Strategies.append(StrategyDirect(self))
        self.Strategies.append(StrategyAlternate(self))
        self.Strategies.append(StrategySide(self))

    def DetermineStratrgy(self, axleState, targetPos : Pos3, obstaclePos : Pos3, steps):
        pos = targetPos
        planeY = targetPos.Y
        armPxy = Pos2.Zero
        obsPxy = obstaclePos.XY
        for i in range(steps):
            pos += self.TargetMotion
            

    def GetTargetAxleState(self, targetPos : Pos3, obstaclePos : Pos3):
        return self.Strategies[self.SelectedStrategy].GetTargetAxleRotation(targetPos, obstaclePos)

    def GetAction(self, axleState, targetPos : Pos3, obstaclePos : Pos3):
        curHash = Utils.GetStateHash(obstaclePos.ToNpArray())
        if curHash != self.StateHash:
            self.StateHash = curHash
            self.Steps = 0
        if self.Steps == 0:
            self.TargetMotion = targetPos
            self.Steps += 1
            return [0, 0, 0, 0, 0, 0]
        if self.Steps == 1:
            self.TargetMotion = targetPos - self.TargetMotion
            self.DetermineStratrgy(targetPos, obstaclePos, 200 - 1)
        action = [0, 1, -1, 0, 0, 0]
        if self.SelectedStrategy == -1:
            target = self.TargetRot
        else:
            target = self.GetTargetAxleState(targetPos, obstaclePos)
        self.ArmStable = True
        for i in range(6):
            action[i] = Utils.GetAxleRotationTransformation(axleState[i], target[i])
            if i != 0 and action[i] > 0.5:
                self.ArmStable = False
        return action
        
    def get_action(self, observation):
        # time.sleep(1) # Add a delay here to clearly see the actions.
        return numpy.array(self.GetAction(observation[0][0:6], Pos3(observation[0][6:9]), Pos3(observation[0][9:12])))

    # region: Round notification functions
    def NotifyTestBegin(self):
        self.Statistics = [[0, 0], [0, 0], [0, 0], 0]
        self.UseStatistics = True
        pass

    def NotifyRoundBegin(self, observation):
        pass

    def NotifyRoundEnd(self, result):
        self.Statistics[self.Strategy][1] += result[-1]
        pass

    def NotifyTestEnd(self):
        print(self.Statistics[3] / 2000)
        # print("\033[96mTest statistics:\n\tS0 Choice =\t{}\n\tS0 Score =\t{}\n\tS1 Choice =\t{}\n\tS1 Score =\t{}".format(self.Statistics[0][0], self.Statistics[0][1] / self.Statistics[0][0], self.Statistics[1][0], self.Statistics[1][1] / self.Statistics[1][0]))
        pass
    # endregion

if __name__ == '__main__':
    import test
    test.main(MyCustomAlgorithm())

# region: fr5 arm parameters
# Segment 1 (base_link to j1_Link): No direct length given relies on joint configuration.
# Segment 2 (j1_Link to j2_Link): 0.152 meters
# Segment 3 (j2_Link to j3_Link): 0.425 meters
# Segment 4 (j3_Link to j4_Link): 0.39501 meters
# Segment 5 (j4_Link to j5_Link): 0.1021 meters
# Segment 6 (j5_Link to j6_Link): 0.102 meters
# Segment 7 (j6_Link to hand_base_link): 0.12 meters
# endregion