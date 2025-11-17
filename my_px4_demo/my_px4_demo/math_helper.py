import math

def sat(x, upper, lower):
    return max(min(x, upper), lower)