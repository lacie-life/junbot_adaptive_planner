from xlrd import open_workbook
import numpy as np
import cv2
from create_map import *
from filter_waypoint import *

'''
simulation() takes .xlsx file as an input
Specificatios of .xlsx file:
Sheet-1 :
	column :     0            1          2        3        4
	          abscissa     ordinate    size_x   size_y   TYPE
	
	TYPE :     0           1             2
	        Obstacle     Source      Destination
Sheet-2 :
   mapsize_x mapsize_y

'''


def simulation(filename):

    # Load the data from the excel file
    alpha = 90000
    book = open_workbook(filename)
    sheet1 = book.sheet_by_index(0)
    sheet2 = book.sheet_by_index(1)
    X = np.asarray(sheet1.col_values(0, start_rowx=0, end_rowx=None), dtype=np.int32)
    Y = np.asarray(sheet1.col_values(1, start_rowx=0, end_rowx=None), dtype=np.int32)
    SIGMA_X = np.asarray(sheet1.col_values(2, start_rowx=0, end_rowx=None))
    SIGMA_Y = np.asarray(sheet1.col_values(3, start_rowx=0, end_rowx=None))
    TYPE = np.asarray(sheet1.col_values(4, start_rowx=0, end_rowx=None))
    map_sx = int(sheet2.cell_value(0, 0))
    map_sy = int(sheet2.cell_value(0, 1))
    grid_sz = 20

    # Create the map
    cv2.namedWindow('path', cv2.WINDOW_NORMAL)
    Map = create_map(map_sx, map_sy, X, Y, SIGMA_X, SIGMA_Y, TYPE)
    img = np.zeros((map_sx, map_sy, 3), np.uint8)
    img = Map * 8000

    # Draw grid
    img_gird_bg = np.zeros((map_sx, map_sy, 3), np.uint8)
    # Draw horizontal lines
    # for i in range(0, img_gird_bg.shape[0], grid_sz):
    #     cv2.line(img_gird_bg, (0, i), (img_gird_bg.shape[1], i), (255, 255, 255), 1)

    # # Draw vertical lines
    # for i in range(0, img_gird_bg.shape[1], grid_sz):
    #     cv2.line(img_gird_bg, (i, 0), (i, img_gird_bg.shape[0]), (255, 255, 255), 1)

    # Combine the map and grid
    # img = cv2.addWeighted(img, 1, img_gird_bg.astype(img.dtype), 0.01, 0)
    img = cv2.cvtColor(img.astype('uint8'), cv2.COLOR_BGR2GRAY)
    heatmap = cv2.applyColorMap(img, cv2.COLORMAP_HSV)
    cv2.imshow('path', heatmap)
    # cv2.waitKey(0)

    # Find the path
    for i in range(len(X)):
        if (TYPE[i] == 2):
            Dx = X[i]
            Dy = Y[i]
        if (TYPE[i] == 1):
            x = x1 = X[i]
            y = y1 = Y[i]
    
    heatmap = draw(heatmap, Dx, Dy, 1)
    count = 0
    cv2.imshow('path', heatmap)
    points = []
    points.append([x, y])

    while (abs(x - Dx) > 5 or abs(y - Dy) > 5):
        heatmap = draw(heatmap, x, y, 0)
        cv2.imshow('path', heatmap)
        cv2.waitKey(10)
        # checking if bot is stationary for long
        if (abs(x - x1) < 1 and abs(y - y1) < 1):
            count += 1
        else:
            count = 0
        # to avoid local minima
        if (count > 10):
            m = 100000
            for i in range(-10, 11):
                for j in range(-10, 11):
                    if (i != 0 or j != 0):
                        if (Map[int(x + i), int(y + j), 1] - Map[int(x), int(y), 1] < m):
                            m = Map[int(x + i), int(y + j), 1] - Map[int(x), int(y), 1]
                            temp0 = x - 1.5 * alpha * m
                            temp1 = y - 1.5 * alpha * m
            x = (temp0)
            y = (temp1)
            count = 0
            points.append([x, y])
            continue
        # gradient descent
        x1 = x
        y1 = y
        temp0 = x - alpha * (Map[int(x) + 3, int(y), 1] - Map[int(x), int(y), 1])
        temp1 = y - alpha * (Map[int(x), int(y) + 3, 1] - Map[int(x), int(y), 1])
        x = (temp0)
        y = (temp1)
        points.append([x, y])

    cv2.imwrite('path.jpg', heatmap)
    cv2.waitKey(1000)
    Map = filter_map(map_sx, map_sy, X, Y, SIGMA_X, SIGMA_Y, TYPE)
    filter_waypoint(points, Map, map_sx, map_sy)
    return


def draw(img, x, y, n):
    x = int(round(x))
    y = int(round(y))
    for i in range(x, x + 6):
        for j in range(y, y + 6):
            img[i, j, n] = 255
    return img


if __name__ == "__main__":
    simulation('Book1.xlsx')
