import numpy as np
import cv2
from pprint import pprint

class cell:
    LocationX = 0
    LocationY = 0
    parent = None
    obstacle = False
    BFS_check = False
    list_check = False

    def __init__(self, y, x, parent):
        self.LocationX = x
        self.LocationY = y
        self.parent = parent

def make_grid(img):
    height, width = img.shape
    grid = [[cell(j,i,None) for i in range(width)] for j in range(height)]
    for i in range(height):
        for j in range(width):
            if img[i, j] != 255:
                grid[i][j].obstacle = True
    return grid
    
if __name__=="__main__": # This main function is only for testing
    img = cv2.imread('./MAP.png', 0)
    ret, bw_img = cv2.threshold(img,0,255,cv2.THRESH_BINARY)
    bw_img = bw_img.astype(np.uint8)
    grid = make_grid(bw_img, 0)
    Dis_to_Goal(grid, [100,100])
    pprint(vars(grid[99][99]))
