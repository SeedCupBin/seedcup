def GetAxleRotationTransformation(current, target):
    return 1 if target - current > 1e-2 else -1 if target - current < -1e-2 else (target - current) / 1e-2;