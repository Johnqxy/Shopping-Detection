# red cube, blue cube and mofang in area A AND B
# xuehua ,dongpeng and gangsiqiu in area C AND D
# ad in area E
import time

import cv2 as cv
import numpy as np
import serial as ser
import Jetson.GPIO as GPIO
from yolo_onnxruntime import yolov5

net = yolov5()
index = 0
LED = 40
type = 'nano1'


def blue(pic,cam_type):
    bluemid = cv.inRange(pic, np.array([90, 130, 100]), np.array([130, 255, 255]))
    cv.imshow(cam_type+'blue-mid', bluemid)
    bluecontours, hierarchy = cv.findContours(bluemid, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    for i in range(len(bluecontours)):
        bluerect = cv.boundingRect(bluecontours[i])
        area = cv.contourArea(bluecontours[i])
        rate = bluerect[2] / bluerect[3]
        print("blue")
        print(area, rate)
        if ((25000 > area > 5000 and 1.5 >= rate > 0.8) or (area>30000 and rate>1.5)) and bluerect[3]<=165:
            cv.rectangle(pic, (bluerect[0], bluerect[1]), (bluerect[0] + bluerect[2], bluerect[1] + bluerect[3]),
                         (255, 0, 0), 2)
            cv.imshow(cam_type+"blue-result", pic)

            return 1


def red(pic,cam_type):
    redlow = cv.inRange(pic, np.array([0, 80, 60]), np.array([10, 256, 256]))
    redhigh = cv.inRange(pic, np.array([170, 90, 60]), np.array([181, 256, 256]))
    redmid = cv.addWeighted(redhigh, 0.5, redlow, 0.5, 0)
    cv.imshow(cam_type+'red-mid', redmid)
    redcontours, hierarchy = cv.findContours(redmid, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    for i in range(len(redcontours)):
        redrect = cv.boundingRect(redcontours[i])
        area = cv.contourArea(redcontours[i])
        rate = redrect[2] / redrect[3]
        print("red")
        print(area, rate)
        if ((25000 > area > 5000 and 1.5 >= rate > 0.8) or (area > 30000 and rate>1.5)) and redrect[3]<=165:
            cv.rectangle(pic, (redrect[0], redrect[1]), (redrect[0] + redrect[2], redrect[1] + redrect[3]),
                         (0, 0, 255), 2)
            cv.imshow(cam_type+"red-result", pic)
            print(True)
            return 1


def control(key, pic_up=None, pic_down=None):
    f.write('----------------------------------\n')
    up_flag = 0
    down_flag = 0
    if key == 97 or key == 98:
        kernel = np.ones((5, 5), np.uint8)
        gaussian_up = cv.GaussianBlur(pic_up, (5, 5), 0)
        opening_up = cv.morphologyEx(gaussian_up, cv.MORPH_RECT, kernel)
        closing_up = cv.morphologyEx(opening_up, cv.MORPH_CLOSE, kernel)
        hsv_up = cv.cvtColor(closing_up, cv.COLOR_BGR2HSV)
        cv.imshow('hsv-up', hsv_up)
        gaussian_down = cv.GaussianBlur(pic_down, (5, 5), 0)
        opening_down = cv.morphologyEx(gaussian_down, cv.MORPH_RECT, kernel)
        closing_down = cv.morphologyEx(opening_down, cv.MORPH_CLOSE, kernel)
        hsv_down = cv.cvtColor(closing_down, cv.COLOR_BGR2HSV)
        if blue(hsv_down,'down') == 1:
            print("bluecube_down")
            f.write('bluecube down\n')
            down_flag += 1
        else:
            print("bluecube_down not found")
            f.write('bluecube_down not found\n')
        if blue(hsv_up,'up') == 1:
            print("bluecube_up")
            f.write('bluecube up\n')
            up_flag += 1
        else:
            f.write('bluecube_up not found\n')
            print("bluecube_up not found")
        if red(hsv_down,'down') == 1:
            print("redcube_down")
            f.write("redcube_down\n")
            down_flag += 1
        else:
            print("redcube_down not found")
        if red(hsv_up,'up') == 1:
            print("redcube_up")
            f.write("redcube_up\n")
            up_flag += 1
        else:
            f.write('redcube_up not found\n')
            print("redcube_up not found")
        if 2 in net.detect(pic_down, 'down'):
            print('mofang_down')
            f.write('mofang_down\n')
            down_flag += 1
        else:
            f.write('mofang_down not found\n')
            print('mofang_down not found')
        if 2 in net.detect(pic_up, 'up'):
            print('mofang_up')
            f.write('mofang_up\n')
            up_flag += 1
        else:
            f.write('mofang_up not found\n')
            print('mofang_up not found')

    if key == 99 or key == 100:
        res_up = net.detect(pic_up, 'up')
        print(res_up)
        if 0 in res_up or 3 in res_up:
            f.write('xuehua_up dongpeng_up\n')
            print('xuehua_up dongpeng_up')
            up_flag += 1
        else:
            f.write('xuehua_up dongpeng_up not found\n')
            print('xuehua_up dongpeng_up not found')
        res_down = net.detect(pic_down, 'down')
        print(res_down)
        if 0 in res_down or 3 in res_down:
            f.write('xuehua_down dongpeng_down\n')
            print('xuehua_down dongpeng_down')
            down_flag += 1
        else:
            f.write('xuehua_down dongpeng_down not found\n')
            print('xuehua_down dongpeng_down not found')
    if key == 101:
        pic_down[:, 400:] = 0
        kernel = np.ones((5, 5), np.uint8)
        gaussian_down = cv.GaussianBlur(pic_down, (5, 5), 0)
        opening_down = cv.morphologyEx(gaussian_down, cv.MORPH_RECT, kernel)
        closing_down = cv.morphologyEx(opening_down, cv.MORPH_CLOSE, kernel)
        hsv_down = cv.cvtColor(closing_down, cv.COLOR_BGR2HSV)
        h, w = pic_down.shape[:2]
        center = (w // 2, h // 2)
        res_down = net.detect(pic_down, 'down')
        if 42 not in res_down:
            down_flag += 1
        M = cv.getRotationMatrix2D(center, 90, 1)
        R90 = cv.warpAffine(pic_down, M, (w, h))
        res_down = net.detect(R90, 'r90')
        if 42 not in res_down:
            down_flag += 1
        M = cv.getRotationMatrix2D(center, 180, 1)
        R180 = cv.warpAffine(pic_down, M, (w, h))
        res_down = net.detect(R180, 'r180')
        if 42 not in res_down:
            down_flag += 1
        M = cv.getRotationMatrix2D(center, 270, 1)
        R270 = cv.warpAffine(pic_down, M, (w, h))
        if red(hsv_down,'down') == 1 or blue(hsv_down,'down') == 1:
            down_flag += 1

        res_down = net.detect(R270, 'r270')
        if 42 not in res_down:
            down_flag += 1
    if up_flag != 0 and down_flag != 0:
        print('all-yes')
        f.write('all-yes\n')
        se.write("0".encode("GB2312"))
    if up_flag != 0 and down_flag == 0:
        print('up-yes down-no')
        f.write('up-yes down-no\n')
        se.write("1".encode("GB2312"))
    if up_flag == 0 and down_flag != 0:
        print('up-no down-yes')
        f.write('up-no down-yes\n')
        se.write("2".encode("GB2312"))
    if up_flag == 0 and down_flag == 0:
        print('all-no')
        f.write('all-no\n')
        se.write("3".encode("GB2312"))
    f.write('----------------------------------\n')


def main():
    a = cv.imread("qxy-6000.jpg")
    net.detect(a, 'test')
    up = cv.VideoCapture(0)
    down = cv.VideoCapture(1)
    GPIO.setmode(GPIO.BOARD)
    GPIO.cleanup(LED)
    GPIO.setup(LED, GPIO.OUT)
    print('start')
    print(up.isOpened(), down.isOpened())
    while up.isOpened() and down.isOpened():
        GPIO.output(LED, GPIO.HIGH)
        ret1, up_frame = up.read()
        ret2, down_frame = down.read()
        up_frame[:100] = 0
        down_frame[:100] = 0
        KeyBoard = cv.waitKey(10)
        res = se.readline().decode("GB2312")
        print(res)
        if KeyBoard & 0xFF == ord('a') or KeyBoard & 0xFF == ord('b') or res == "a":
            cv.imwrite(type+'-up-'+time.strftime("%H-%M-%S",time.localtime())+'.jpg',up_frame)
            cv.imwrite(type+'-down-'+time.strftime("%H-%M-%S",time.localtime())+'.jpg',down_frame)
            print(KeyBoard)
            KeyBoard = 97
            control(KeyBoard, up_frame, down_frame)
        if KeyBoard & 0xFF == ord('c') or KeyBoard & 0xFF == ord('d') or res == "b":
            cv.imwrite(type+'-up-'+time.strftime("%H-%M-%S",time.localtime())+'.jpg',up_frame)
            cv.imwrite(type+'-down-'+time.strftime("%H-%M-%S",time.localtime())+'.jpg',down_frame)
            KeyBoard = 100
            print(KeyBoard)
            control(KeyBoard, up_frame, down_frame)
        if KeyBoard & 0xFF == ord('e') or res == "e":
            cv.imwrite(type+'-down-'+time.strftime("%H-%M-%S",time.localtime())+'.jpg',down_frame)
            KeyBoard = 101
            control(KeyBoard, pic_down=down_frame)

        cv.imshow("up", up_frame)
        cv.imshow("down", down_frame)
    down.release()
    up.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    with open('history.txt', 'w') as f:
        try:
            se = ser.Serial("/dev/ttyUSB0", 115200, timeout=0.01)
            main()
        except Exception as e:
            print(e)
            GPIO.output(LED, GPIO.LOW)
