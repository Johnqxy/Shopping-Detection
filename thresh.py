import cv2 as cv
import numpy as np
def blue(pic):
    bluemid = cv.inRange(pic, np.array([90, 85, 70]), np.array([125, 256, 256]))
    cv.imshow("blue", bluemid)
    bluecontours, hierarchy = cv.findContours(bluemid, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    for i in range(len(bluecontours)):
        n = 0
        bluerect = cv.boundingRect(bluecontours[i])
        area = cv.contourArea(bluecontours[i])
        rate = bluerect[2] / bluerect[3]
        print("blue")
        print(area, rate)
        if 25000 > area > 10000 and 1.5 >= rate > 0.8:
            cv.rectangle(pic, (bluerect[0], bluerect[1]), (bluerect[0] + bluerect[2], bluerect[1] + bluerect[3]),
                         (255, 0, 0), 2)
            cv.imshow("blue result", pic)
            print(True)
            return 1
        n += 1
    cv.waitKey(0)


def red(pic):
    redlow = cv.inRange(pic, np.array([-1, 80, 60]), np.array([20, 256, 256]))
    redhigh = cv.inRange(pic, np.array([170, 90, 60]), np.array([181, 256, 256]))
    redmid = cv.addWeighted(redhigh, 0.5, redlow, 0.5, 0)
    cv.imshow("red", redmid)
    redcontours, hierarchy = cv.findContours(redmid, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    for i in range(len(redcontours)):
        n = 0
        redrect = cv.boundingRect(redcontours[i])
        area = cv.contourArea(redcontours[i])
        rate = redrect[2] / redrect[3]
        print("red")
        print(area, rate)
        if 25000 > area > 10000 and 1.5 >= rate > 0.8:
            cv.rectangle(pic, (redrect[0], redrect[1]), (redrect[0] + redrect[2], redrect[1] + redrect[3]),
                         (0, 0, 255), 2)
            cv.imshow("red result", pic)
            print(True)
            return 1
        n += 1
    cv.waitKey(0)
img=cv.imread('4.jpg')
cv.imshow('4',img)
img=cv.cvtColor(img,cv.COLOR_BGR2HSV)
blue(img)