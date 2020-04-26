import cv2
import numpy as np
import math
import re
import scipy.stats as st
    
def gkern(kernlen, nsig):
    """Returns a 2D Gaussian kernel array."""

    interval = (2*nsig+1.)/(kernlen)
    x = np.linspace(-nsig-interval/2., nsig+interval/2., kernlen+1)
    kern1d = np.diff(st.norm.cdf(x))
    kernel_raw = np.sqrt(np.outer(kern1d, kern1d))
    kernel = kernel_raw/kernel_raw.sum()
    return kernel

def grid_maker(path):
    fin = open(path, 'r')
    f = fin.readlines()
    newList = []
    foundList = []
    link_mark = False
    size_mark = False
    visual_mark = False
    pose_mark = False
    pi = math.pi
    img = np.zeros((400,1000))
    img = np.zeros((400,1000,1), np.uint8)
    
    for line in f:
        if "link name=" in line and not link_mark:
            link_mark = True
            current_link = re.search("'(.+?)'", line).group(1)
            if "Wall" not in current_link:
                link_mark = False
        if "size" in line and not size_mark and link_mark:
            current_size = [float(s) for s in re.findall(r'-?\d+\.?\d*', line)]
            size_mark = True
        if "/visual" in line and size_mark:
            visual_mark = True
        if "pose frame=" in line and visual_mark:
            current_pose = [float(s) for s in re.findall(r'-?\d+\.?\d*', line)]
            foundList.append(current_pose[:-4]+[round((current_pose[-1]*180.0)/pi, 1)]+current_size[:-1])
            pose_mark = True
        if pose_mark:
            link_mark = False
            size_mark = False
            visual_mark = False
            pose_mark = False
    for link in foundList:
        #link [x, y, theta, h, t]
        #print link
        t = round(link[4]*100)
        h = round(link[3]*100)
        x_axis = round((link[0] + 5.0)*100)
        y_axis = round((2.0-link[1])*100)
        if abs(link[2]) == 90:
            pos_x = x_axis - (t/2)
            pos_y = y_axis - (h/2)
            j = t
            i = h
        else:
            pos_x = x_axis - (h/2)
            pos_y = y_axis - (t/2)
            j = h
            i = t
        i = int(i)
        j = int(j)
        pos_x = int(pos_x)
        pos_y = int(pos_y)
        # print [pos_x, pos_y, h, t]
        for count_x in range(j):
            for count_y in range(i):
                img[pos_y+count_y-1, pos_x+count_x] = 255
        
    return img
    
if __name__ == "__main__":
    img = grid_maker("../worlds/ms4_world.world")
    e = gkern(5, 3)*1700
    print e
    cv2.imshow("Map", img)
    cv2.imwrite("MAP.png", img)
    cv2.waitKey(0)
    cv2.DestroyAllWindows()
    
