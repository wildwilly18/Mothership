from mothership import *
# ROS python API
import roslib
import rospy
import cv2
import numpy as np

def main():

    logName = input("Input Log Name: ")
    logName = logName + '.txt'
    print("Log Named: " + logName)
    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()


    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    # Making note here:
    #                   Chase Craft: uav0, iris0
    #                    Mothership: uav1, iris1: Camera not used

    rospy.Subscriber('uav0/mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('uav0/mavros/local_position/pose', PoseStamped, cnt.posCb)
    
    # Subscribe to chase drone's global position
    rospy.Subscriber('uav0/mavros/global_position/global', NavSatFix, cnt.globalLoc)

    # Subscribe to drone's global heading
    rospy.Subscriber('uav0/mavros/global_position/compass_hdg', Float64, cnt.updateHDG)

    # Subscribe to the camera image we want to take a look at
    rospy.Subscriber('iris0/usb_cam/image_raw/compressed', CompressedImage, cnt.updateVisLoc, queue_size=1)
    rospy.Subscriber('iris0/usb_cam/image_raw', CompressedImage, cnt.updateVisLoc, queue_size=1)

    # Subscribe to mothership drone's global position, update callback fnc
    rospy.Subscriber('uav1/mavros/global_position/global', NavSatFix, cnt.updateMothershipLoc)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('uav0/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    
    logFile = open(logName, 'w')
    
    printheader = cnt.logData('header')
    print(printheader)

    logFile.writelines(printheader)
    cnt.alg.algo_counter = 1000

    # ROS main loop
    while not rospy.is_shutdown():
        while not(cnt.alg.visual_mode):
            cnt.determineVisualAlg()
            cnt.determineEnterVisualMode()

            loc_string = str('X-Dist: ') + str(cnt.alg.x_vis_err) + str(' Y-Dist: ') + str(cnt.alg.y_vis_err) + str(' z-dist: ') + str(cnt.alg.z_vis_err)
            print(loc_string)

            if(cnt.alg.visual_mode):
                print('Entering Visual Mode')
            line = cnt.logData()
            logFile.writelines(line)
            rate.sleep()

        while(cnt.alg.visual_mode):
            cnt.determineVisualAlg()
            cnt.determineExitVisualMode()
            cnt.checkVisAppInRadius()
            cnt.updateVisualDist()

            loc_string = str('X-Dist: ') + str(cnt.alg.x_vis_err) + str(' Y-Dist: ') + str(cnt.alg.y_vis_err) + str(' z-dist: ') + str(cnt.alg.z_vis_err)
            print(loc_string)

            if not(cnt.alg.visual_mode):
                  print('Exiting Visual Mode')
            line = cnt.logData()
            logFile.writelines(line)
            rate.sleep()

def handleROSException():
    pass  

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:handleROSException()