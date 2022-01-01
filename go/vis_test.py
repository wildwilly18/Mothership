from mothership import *
# ROS python API
import roslib
import rospy
import cv2
import numpy as np
import time

def main():

    vid = cv2.VideoCapture(-1)

    cnt = Controller()

    while (True):
        ret,frame = vid.read()

        cnt.imgCap(frame)

        cnt.updateVisLoc(cnt.alg.img)


if __name__ == '__main__':
    main()