from mothership import *
# ROS python API
import roslib
import rospy
import cv2
import numpy as np
import time

def main():

    vid = cv2.VideoCapture(0)

    cnt = Controller()

    while (True):
        ret,frame = vid.read()
        if not ret:
            print("failed to grab image")
            break
        else:
            #print(frame.shape)
            cnt.imgCallback(frame)
            cnt.updateVisLoc(cnt.alg.img)
            cnt.determineVisualAlg(cnt.alg.img)
            print("Alg Counter: " + str(cnt.alg.vis_counter))

if __name__ == '__main__':
    main()