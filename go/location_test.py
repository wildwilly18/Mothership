from mothership import *
# ROS python API
import roslib
import rospy
import cv2
import numpy as np
import time

def main():

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
    #                   Chase Craft: uav1, iris0
    #                    Mothership: uav0, iris1: Camera not used

    rospy.Subscriber('uav1/mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('uav1/mavros/local_position/pose', PoseStamped, cnt.posCb)
    
    # Subscribe to chase drone's global position
    rospy.Subscriber('uav1/mavros/global_position/global', NavSatFix, cnt.globalLoc)

    # Subscribe to drone's global heading
    rospy.Subscriber('uav1/mavros/global_position/compass_hdg', Float64, cnt.updateHDG)

    # Subscribe to the camera image we want to take a look at
    #rospy.Subscriber('iris0/usb_cam/image_raw/compressed', CompressedImage, cnt.locateAruco, queue_size=10)

    # Subscribe to mothership drone's global position, update callback fnc
    rospy.Subscriber('uav0/mavros/global_position/global', NavSatFix, cnt.updateMothershipLoc)

    # Setpoint publisher    
    #sp_pub = rospy.Publisher('uav0/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    cnt.initLatLon()
    rate.sleep()

    print("Initializing Lat Lon Origin:")
    while (True):

        cnt.updateRendesvousLoc()
        print('X Error: ' + str(cnt.alg.rs_target_x - cnt.local_pos.x) + ' Y Error: ' + str(cnt.alg.rs_target_y - cnt.local_pos.y) + ' Z Error: ' + str(cnt.alg.rs_target_z - cnt.local_pos.z))
        rate.sleep()


if __name__ == '__main__':
    main()