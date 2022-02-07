from mothership import *
# ROS python API
import roslib
import rospy
import cv2
import numpy as np

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
    rospy.Subscriber('mavros/state', State, cnt.stateCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    
    # Subscribe to drone's global position
    rospy.Subscriber('mavros/global_position/global', NavSatFix, cnt.globalLoc)

    # Subscribe to drone's global heading
    rospy.Subscriber('mavros/global_position/compass_hdg', Float64, cnt.updateHDG)

    # Subscribe to the Motherships Global Position
    rospy.Subscriber('Mothership_Position/mav1pos', NavSatFix, cnt.mshipCb)


    # Subscribe to the camera image we want to take a look at
    #rospy.Subscriber('iris/usb_cam/image_raw/compressed', CompressedImage, cnt.locateAruco, queue_size=10)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    print 'Pub Sub setup. Trying to arm'
    cnt.printLoc()
    # Make sure the drone is armed
    while not cnt.state.armed:
        
        modes.setArm()
        cnt.printLoc()
        #print 'armed'
        rate.sleep()

    # set in takeoff mode and takeoff to default altitude (3 m)
    modes.setTakeoff()
    rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()

    while not rospy.is_shutdown():            
        modes.setAutoLandMode


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
