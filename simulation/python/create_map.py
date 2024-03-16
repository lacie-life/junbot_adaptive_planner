import numpy as np
from field import rep_field
from field import att_field
from field import guassian
import cv2
import math


def create_map(mx, my, X, Y, SIGMA_X, SIGMA_Y, TYPE):
    n = len(X)
    Map = np.zeros((mx, my, 3))
    Map_R = np.zeros((mx, my))
    Map_R = Map_R + 10
    Map_B = np.zeros((mx, my))
    Map_B = Map_B + 100

    # Create the map with repulsive and attractive fields in channel 1
    
    inter = [] # intersection between position robot with center pixel
    for i in range(n):
        if (TYPE[i] == 0):
            posRbX, posRbY = [50, 250]
            dis = math.sqrt((posRbX-Y[i])*(posRbX-Y[i])+(posRbY-X[i])*(posRbY-X[i]))
            dis = (1/math.log(dis))*750
            print (dis)
            Map[:, :, 0] += 15 * rep_field(X[i], Y[i], dis, dis, mx, my)
            centerX = Y[i]
            centerY = X[i]
            for Cx in range(60, 350, 1):
                intersection = find_point_A((50, 250), (centerX, centerY), Cx)
                temp1X, temp1Y = intersection
                if Map[temp1Y, temp1X, 0] > 1:
                    inter.append(intersection)
                    break
        elif (TYPE[i] == 2):
            Map[:, :, 0] += att_field(X[i], Y[i], mx, my)
    # Color encode the map for visualization
    Map[:, :, 2] = Map[:, :, 0] * 0.5
    Map[:, :, 1] = Map[:, :, 0] * 0.2
    
    return Map


def filter_map(mx, my, X, Y, SIGMA_X, SIGMA_Y, TYPE):
    n = len(X)
    Map = np.zeros((mx, my, 3))
    for i in range(n):
        if (TYPE[i] == 0):
            Map[:, :, 2] += 2 * rep_field(X[i], Y[i], SIGMA_X[i], SIGMA_Y[i], mx, my)
    return Map

def find_point_A(B, C, Ax):
    # Lấy tọa độ của điểm B và C
    x1, y1 = B
    x2, y2 = C
    # Tính giá trị y tương ứng với Ax trên đường thẳng BC
    slope = (y2 - y1) / (x2 - x1)
    Ay = round(slope * (Ax - x1) + y1)
    
    return (Ax, Ay)



