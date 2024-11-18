import math

def GetAxleRotationTransformation(current, target):
    return 1 if target - current > 1e-2 else -1 if target - current < -1e-2 else (target - current) / 1e-2
def GetRectangularDistance(dis1, dis2):
    return math.sqrt(dis1 * dis1 + dis2 * dis2)
def GetTriangleArea(a, b, c):
    return math.sqrt((a + b + c) * (a + b - c) * (b + c - a) * (c + a - b)) / 4
def GetTriangleAngle(area, a, b):
    return math.asin(2 * area / a / b)
def AngleToPercentage(angle):
    angle /= math.pi
    angle /= 2
    while angle < 0: angle += 1
    return angle