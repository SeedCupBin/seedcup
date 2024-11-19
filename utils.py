import math

def GetAxleRotationTransformation(current, target):
    return 1 if target - current > 1e-2 else -1 if target - current < -1e-2 else (target - current) / 1e-2

def GetRectangularDistance(dis1, dis2):
    return math.sqrt(dis1 * dis1 + dis2 * dis2)

# def GetTriangleArea(a, b, c):
#     return math.sqrt((a + b + c) * (a + b - c) * (b + c - a) * (c + a - b)) / 4

def GetTriangleAngle(c, a, b):
    return math.acos((a * a + b * b - c * c) / 2 / a / b)

def NormalizeAngle(angle):
    angle /= 2 * math.pi
    angle -= math.floor(angle)
    return angle