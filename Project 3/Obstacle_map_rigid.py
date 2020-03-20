import numpy as np
import math
import matplotlib.pyplot as plt

def cart2img(ix):
    return [ix[0], ix[1]]

def point_robot_obstacle_space(x, y):
    obstacle = False

    if ((x - math.ceil(225)) ** 2 + math.ceil(y - (150)) ** 2 - math.ceil(25) ** 2) <= 0:  # circle
        obstacle = True

    if ((x - math.ceil(150)) / math.ceil(40)) ** 2 + ((y - math.ceil(100)) / math.ceil(20)) ** 2 - 1 <= 0:  # ellipse
        obstacle = True

    if (5 * y + 3 * x - 725 >= 0) and (5 * y - 3 * x + 475 <= 0) and (5 * y + 3 * x - 875 <= 0) and (
            5 * y - 3 * x + 625 >= 0):  # rhomboid
        obstacle = True

    if (65 * y + 37 * x - 5465 >= 0) and (5 * y - 9 * x - 65 <= 0) and (65 * y + 37 * x - 6235 <= 0) and (
            5 * y - 9 * x + 705 >= 0):  # rectangle rotated 30 degrees
        obstacle = True

    # 6-Side Polygon has been split into 2 parts: right side and left side
    if (5 * y + 6 * x - 1050 >= 0) and (5 * y - 6 * x - 150 >= 0) and (5 * y + 7 * x - 1450 <= 0) and (
            5 * y - 7 * x - 400 <= 0):  # right side of polygon
        obstacle = True

    if (y - x - 100 >= 0) and (5 * y - 65 * x + 700 <= 0) and (y - 185 <= 0) and (
            5 * y - 7 * x - 400 >= 0):  # left side of polygon
        obstacle = True

    return obstacle

def obs():
    a = np.zeros((200,300),np.uint8)
    r = 0
    c = 1
    for i in range(0,299):
        for j in range(0,199):
            idx = cart2img([i,j])
            if point_robot_obstacle_space(idx[0],idx[1]) == True:
                a[j,i]=1
    # a= np.transpose(a)
    return np.asarray(a)

def main():
    a = obs()
    figure, ax = plt.subplots()
    plt.imshow(a)
    ax.set_ylim(bottom=0, top=200)
    ax.set_xlim(left=0, right=300)
    plt.show()


if __name__=="__main__":
    main()






