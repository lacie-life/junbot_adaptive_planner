import numpy as np
from field import rep_field
from field import att_field
from field import guassian


def create_map(mx, my, X, Y, SIGMA_X, SIGMA_Y, TYPE):
    n = len(X)
    Map = np.zeros((mx, my, 3))
    Map_R = np.zeros((mx, my))
    Map_R = Map_R + 10
    Map_B = np.zeros((mx, my))
    Map_B = Map_B + 100

    # Create the map with repulsive and attractive fields in channel 1
    for i in range(n):
        if (TYPE[i] == 0):
            Map[:, :, 0] += 15 * rep_field(X[i], Y[i], SIGMA_X[i], SIGMA_Y[i], mx, my)
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



