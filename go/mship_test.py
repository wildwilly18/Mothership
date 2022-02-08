from mothership import *
# ROS python API
import roslib
import rospy
import cv2
import numpy as np

def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=False)

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
    rospy.Subscriber('mav1pos', NavSatFix, cnt.mshipCb)

    # Subscribe to the camera image we want to take a look at
    #rospy.Subscriber('iris/usb_cam/image_raw/compressed', CompressedImage, cnt.locateAruco, queue_size=10)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    print 'Establishing a connection to the Mothership Position.'
    print 'Initializing mothership local positioning'
    cnt.initializeLocalZero()

    cnt.mship.init_Mothership_origin(cnt.lat0, cnt.lon0, cnt.alt0)
    
    while 1:
        cnt.mship.update_Mothership_location(cnt.mship_lat, cnt.mship_lon, cnt.mship_alt, verbose=1)
        rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
