import numpy as np
from field import rep_field
from field import att_field
from field import guassian
import cv2
import math
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.pyplot as plt


def create_map(mx, my, X, Y, SIGMA_X, SIGMA_Y, TYPE):
    n = len(X)
    Map = np.zeros((mx, my, 3))

    # Create the map with repulsive and attractive fields in channel 1
    
    inter = [] # intersection between position robot with center pixel
    for i in range(n):
        if (TYPE[i] == 0):
            posRbX, posRbY = [50, 250]
            dis = math.sqrt((posRbX-Y[i])*(posRbX-Y[i])+(posRbY-X[i])*(posRbY-X[i]))
            dis = (1/math.log(dis))*750
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
    hull_temp = []
    for i in range(Map.shape[0]):
        for j in range(Map.shape[1]):
            if Map[i, j, 0] > 1.3 and Map[i, j, 0] < 10:
                hull_temp.append([j, i])

    hull_temp = np.array(hull_temp)
    hull = ConvexHull(hull_temp)
    # fig, (ax1, ax2) = plt.subplots(ncols=2, figsize=(10, 3))

    # for ax in (ax1, ax2):
    #     ax.plot(hull_temp[:, 0], hull_temp[:, 1], '.', color='k')
    #     if ax == ax1:
    #         ax.set_title('Given points')
    #     else:
    #         ax.set_title('Convex hull')
    #         for simplex in hull.simplices:
    #             ax.plot(hull_temp[simplex, 0], hull_temp[simplex, 1], 'c')
    #         ax.plot(hull_temp[hull.vertices, 0], hull_temp[hull.vertices, 1], 'o', mec='r', color='none', lw=1, markersize=10)
    #     ax.set_xticks(range(10))
    #     ax.set_yticks(range(10))
    # plt.show()
    
    
    Map[:, :, 2] = Map[:, :, 0] * 0.2
    Map[:, :, 1] = Map[:, :, 0] * 0.05
    
    return Map, hull, hull_temp


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



