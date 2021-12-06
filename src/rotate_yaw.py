from mothership import *
# ROS python API
import roslib
import rospy
import cv2
import numpy as np
import time

def main():

    #Get a start time of script
    start_time = time.clock()



    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    #Open up a file to write the data too.
    logFile = open("out.txt", 'w')
    
    printheader = cnt.logData('header')

    logFile.writelines(printheader)

    # Mothership object to track
    #mShip = Mothership()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    
    # Subscribe to drone's global position
    rospy.Subscriber('mavros/global_position/global', NavSatFix, cnt.globalLoc)

    # Subscribe to drone's global heading
    rospy.Subscriber('mavros/global_position/compass_hdg', Float64, cnt.updateHDG)

    # Subscribe to the camera image we want to take a look at
    rospy.Subscriber('iris/usb_cam/image_raw/compressed', CompressedImage, cnt.imgCallback, queue_size=10)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    print 'Pub Sub setup. Trying to arm'
    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        print 'armed'
        rate.sleep()

    # set in takeoff mode and takeoff to default altitude (3 m)
    modes.setTakeoff()
    rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        loggedData = cnt.logData()
        print loggedData
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()
    yaw = 0.25
    # ROS main loop
    while not rospy.is_shutdown():
        yaw = yaw + 0.000005
        cnt.updateSp(1, 1, 1, yaw)
        
        #Update the set point of the quad rotor.
        sp_pub.publish(cnt.sp)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
