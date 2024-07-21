from xlrd import open_workbook
import numpy as np
import cv2
from create_map import *
from filter_waypoint import *
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import os

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
    # ROBOT
    X = np.asarray(sheet1.col_values(0, start_rowx=0, end_rowx=None), dtype=np.int32) 
    Y = np.asarray(sheet1.col_values(1, start_rowx=0, end_rowx=None), dtype=np.int32)
    #size of the robot
    SIGMA_X = np.asarray(sheet1.col_values(2, start_rowx=0, end_rowx=None))
    SIGMA_Y = np.asarray(sheet1.col_values(3, start_rowx=0, end_rowx=None))
    TYPE = np.asarray(sheet1.col_values(4, start_rowx=0, end_rowx=None))
    map_sx = int(sheet2.cell_value(0, 0))
    map_sy = int(sheet2.cell_value(0, 1))
    grid_sz = 20

    # Create the map
    cv2.namedWindow('path', cv2.WINDOW_NORMAL)
    Map, hull, hull_temp = create_map(map_sx, map_sy, X, Y, SIGMA_X, SIGMA_Y, TYPE)
    
    img = np.zeros((map_sx, map_sy, 3), np.uint8)
    img = Map.copy()
    # for i in range(len(X)):
    #     cv2.circle(img,(Y[i], X[i]), 2, (0, 255, 128), 1)
    # print(hull_temp[hull.vertices, 0])
    # for i in range(hull_temp[hull.vertices, 0].shape[0]-1):
    #     cv2.line(img, (int(hull_temp[hull.vertices, 0][i]), int(hull_temp[hull.vertices, 1][i])), (int(hull_temp[hull.vertices, 0][i+1]), int(hull_temp[hull.vertices, 1][i+1])), (0,255,0), 1)


    # Draw the map
    

    # Draw grid
    img_gird_bg = np.zeros((map_sx, map_sy, 3), np.uint8)
    # Draw horizontal lines
    # for i in range(0, img.shape[0], grid_sz):
    #     cv2.line(img, (0, i), (img.shape[1], i), (0, 255, 128), 1)
    
    # # Draw vertical lines
    # for i in range(0, img.shape[1], grid_sz):
    #     cv2.line(img, (i, 0), (i, img.shape[0]), (0, 255, 128), 1)
    # cv2.line(img, (125, 275), (325, 275), (0, 255, 128), 1)
    # cv2.line(img, (125, 375), (325, 375), (0, 255, 128), 1)
    # for i in range(125, 301, 25):
    #     cv2.line(img, (i, 275), (i, 375), (0, 255, 128), 1)
    # cv2.line(img, (125, 375), (125, 275), (0, 255, 128), 1)
    # cv2.line(img, (325, 275), (325, 375), (0, 255, 128), 1)
    # for i in range(275, 376, 25):
    #     cv2.line(img, (125, i), (325, i), (0, 255, 128), 1)
    # Combine the map and grid
    # img = cv2.addWeighted(img, 1, img_gird_bg.astype(img.dtype), 0.1, 0)
    # img = cv2.cvtColor(img.astype('uint8'), cv2.COLOR_BGR2GRAY)
    # heatmap = cv2.applyColorMap(img, cv2.COLORMAP_JET)
    # heatmap = cv2.addWeighted(img, 1, img_gird_bg.astype(img.dtype), 0.5, 0.1)
    # cv2.rectangle(img,(35, 240), (55, 260), (255,0,0), -1)
    # cv2.circle(img,(450,425), 6, (0,0,255), -1)
    # cv2.circle(img,(150, 300), 2, (0, 255, 128), 1)
    # cv2.imwrite("APF_sim_python.png",img.astype('uint8'))

    cv2.line(img, (3, 3), (497, 3), (255, 255, 255), 3)
    cv2.line(img, (497, 497), (497, 3), (255, 255, 255), 3)
    cv2.line(img, (497, 497), (3, 497), (255, 255, 255), 3)
    cv2.line(img, (3, 3), (3, 497), (255, 255, 255), 3)
    
    cv2.line(img, (375, 255), (497, 255), (255, 255, 255), 3)

    cv2.line(img, (300, 255), (497, 255), (255, 255, 255), 3)
    cv2.line(img, (300, 255), (300, 375), (255, 255, 255), 3)
    
    cv2.line(img, (150, 255), (3, 255), (255, 255, 255), 3)
    cv2.line(img, (150, 255), (150, 400), (255, 255, 255), 3)
    cv2.line(img, (150, 450), (150, 500), (255, 255, 255), 3)

    # obstacle chair
    cv2.line(img, (70,380), (70,345), (0, 0, 255), 3)
    cv2.line(img, (80,380), (80,345), (0, 0, 255), 3)
    cv2.line(img, (70,380), (80,380), (0, 0, 255), 3)
    cv2.line(img, (70,345), (80,345), (0, 0, 255), 3)
    # cv2.line(img1, (75,350), (180,255), (0, 0, 255), 3)
    # cv2.line(img1, (250,255), (250,225), (0, 0, 255), 3)
    # cv2.line(img1, (180,225), (250,225), (0, 0, 255), 3)

    # cv2.circle(img1,(175,225), 2, (0,0,0), 4)
    # cv2.circle(img1,(175,255), 2, (0,0,0), 4)
    # cv2.circle(img1,(225,225), 2, (0,0,0), 4)
    # cv2.circle(img1,(225,255), 2, (0,0,0), 4)
    # #obstacle table
    # cv2.line(img, (205,225), (205,255), (0, 0, 255), 3)
    # cv2.line(img, (275,255), (205,255), (0, 0, 255), 3)
    # cv2.line(img, (275,255), (275,225), (0, 0, 255), 3)
    # cv2.line(img, (205,225), (275,225), (0, 0, 255), 3)

    #obstacle person
    cv2.line(img, (210,230), (210,250), (0, 0, 255), 2)
    cv2.line(img, (230,250), (210,250), (0, 0, 255), 2)
    cv2.line(img, (230,250), (230,230), (0, 0, 255), 2)
    cv2.line(img, (210,230), (230,230), (0, 0, 255), 2)


    # robot
    # cv2.line(img1, (30,30), (30,60), (0, 255, 0), 3)
    # cv2.line(img1, (50,60), (30,60), (0, 255, 0), 3)
    # cv2.line(img1, (50,60), (50,30), (0, 255, 0), 3)
    # cv2.line(img1, (30,30), (50,30), (0, 255, 0), 3)
    cv2.circle(img,(205,195), 10, (0,255,0), 3)
    # target
    cv2.circle(img,(450,350), 4, (255, 255, 255), 3)

    cv2.imshow('path', img)

    # cv2.setMouseCallback('path', save_pixel)
    cv2.waitKey(0)

    # # Find the path
    # for i in range(len(X)):
    #     if (TYPE[i] == 2):
    #         Dx = X[i]
    #         Dy = Y[i]
    #     if (TYPE[i] == 1):
    #         x = x1 = X[i]
    #         y = y1 = Y[i]
    
    # heatmap = draw(heatmap, Dx, Dy, 1)
    # count = 0
    # cv2.imshow('path', heatmap)
    # points = []
    # points.append([x, y])


    # cv2.namedWindow('map', cv2.WINDOW_NORMAL)
    # img1 = np.full((map_sx, map_sy, 3), 255).astype('uint8')
    # cv2.line(img1, (3, 3), (497, 3), (0, 0, 0), 3)
    # cv2.line(img1, (497, 497), (497, 3), (0, 0, 0), 3)
    # cv2.line(img1, (497, 497), (3, 497), (0, 0, 0), 3)
    # cv2.line(img1, (3, 3), (3, 497), (0, 0, 0), 3)
    
    # cv2.line(img1, (375, 255), (497, 255), (0, 0, 0), 3)

    # cv2.line(img1, (300, 255), (497, 255), (0, 0, 0), 3)
    # cv2.line(img1, (300, 255), (300, 375), (0, 0, 0), 3)
    
    # cv2.line(img1, (150, 255), (3, 255), (0, 0, 0), 3)
    # cv2.line(img1, (150, 255), (150, 400), (0, 0, 0), 3)
    # cv2.line(img1, (150, 450), (150, 500), (0, 0, 0), 3)

    # # obstacle chair
    # cv2.line(img1, (70,380), (70,345), (0, 0, 255), 3)
    # cv2.line(img1, (80,380), (80,345), (0, 0, 255), 3)
    # cv2.line(img1, (70,380), (80,380), (0, 0, 255), 3)
    # cv2.line(img1, (70,345), (80,345), (0, 0, 255), 3)
    # # cv2.line(img1, (75,350), (180,255), (0, 0, 255), 3)
    # # cv2.line(img1, (250,255), (250,225), (0, 0, 255), 3)
    # # cv2.line(img1, (180,225), (250,225), (0, 0, 255), 3)

    # # cv2.circle(img1,(175,225), 2, (0,0,0), 4)
    # # cv2.circle(img1,(175,255), 2, (0,0,0), 4)
    # # cv2.circle(img1,(225,225), 2, (0,0,0), 4)
    # # cv2.circle(img1,(225,255), 2, (0,0,0), 4)
    # #obstacle table
    # cv2.line(img1, (180,225), (180,255), (0, 0, 255), 3)
    # cv2.line(img1, (250,255), (180,255), (0, 0, 255), 3)
    # cv2.line(img1, (250,255), (250,225), (0, 0, 255), 3)
    # cv2.line(img1, (180,225), (250,225), (0, 0, 255), 3)
    # # robot
    # # cv2.line(img1, (30,30), (30,60), (0, 255, 0), 3)
    # # cv2.line(img1, (50,60), (30,60), (0, 255, 0), 3)
    # # cv2.line(img1, (50,60), (50,30), (0, 255, 0), 3)
    # # cv2.line(img1, (30,30), (50,30), (0, 255, 0), 3)
    # cv2.circle(img1,(40,40), 10, (0,255,0), 3)
    # # target
    # cv2.circle(img1,(450,350), 4, (0,0,0), 3)
    # cv2.imshow('map', img1.astype('uint8'))
    # cv2.waitKey(0)
    return


def draw(img, x, y, n):
    x = int(round(x))
    y = int(round(y))
    for i in range(x, x + 6):
        for j in range(y, y + 6):
            img[i, j, n] = 255
    return img


if __name__ == "__main__":
    simulation('/home/gn/Github/planner_ws/src/junbot_adaptive_planner/simulation/python/Book1 copy 4.xlsx') # for cells
    # simulation('/home/gn/Github/planner_ws/src/junbot_adaptive_planner/simulation/python/Book1 copy.xlsx') # for object
