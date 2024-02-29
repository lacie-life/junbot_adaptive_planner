import math
import numpy as np

def att_field(px, py, mx, my):
    res = np.zeros((mx, my))
    for x in range(mx):
        for y in range(my):
            t = 20
            d = math.sqrt((x - px) ** 2 + (y - py) ** 2)
            if (d > t):
                res[x, y] = t * d - 0.5 * (t ** 2)
            else:
                res[x, y] = 0.5 * (d ** 2)
            res[x, y] = 0.0001 * res[x, y]
            # print(res[x, y])
    return res


def rep_field(px, py, sx, sy, mx, my):
    res = np.zeros((mx, my))
    tempX = 50
    tempY = 250
    for x in range(mx):
        for y in range(my):
            thr = 5
            d = ((x - px) ** 2) / sx ** 2 + ((y - py) ** 2) / sy ** 2
            d = math.sqrt(d)
            # Drawing the repulsive field
            if (d < 1 and d != 0):
                vector1 = np.array([x - px, y - py])
                vector2 = np.array([tempX - 300, tempY - 250])
                # Tính góc giữa hai vector (trong đơn vị radian)
                theta = np.arccos(np.dot(vector1, vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2)))
                # if theta <= (math.pi/2):
                res[x, y] = (300 + 300 * (math.cos(theta))) * (1 - 1 / d) ** 2
                # else:
                #     res[x, y] = (1 - 1 / d) ** 2
                res[x, y] = res[x, y] * 0.0001
                # print(res[x, y])
            else:
                res[x, y] = 0
    return res

def guassian(px, py, sx, sy, mx, my):
    res = np.zeros((mx, my))
    for x in range(mx):
        for y in range(my):
            d = ((x - px) ** 2) / sx ** 2 + ((y - py) ** 2) / sy ** 2
            d *= -0.5
            amp = 1 / (2 * (math.pi) * sx * sy)
            res[x, y] = 200 * amp * math.exp(d)
    return res


