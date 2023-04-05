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
    rospy.Subscriber('iris0/usb_cam/image_raw/compressed', CompressedImage, cnt.locateAruco, queue_size=10)

    # Subscribe to mothership drone's global position, update callback fnc
    rospy.Subscriber('uav1/mavros/global_position/global', NavSatFix, cnt.updateMothershipLoc)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('uav0/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    modes.setPositionMode()
    rate.sleep()

    x_cmd = cnt.local_pos.x
    y_cmd = cnt.local_pos.y
    z_cmd = cnt.local_pos.y
    cnt.updateSp(x_cmd, y_cmd, z_cmd)

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    for i in range(100):
        sp_pub.publish(cnt.sp)
        rate.sleep()

    # activate OFFBOARD mode
    modes.setOffboardMode()

    print('OFFBOARD MODE SET...')
    print('ARMING')
    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    print('ARMED')
    print('TAKING OFF')
    # ROS main loop
    while not rospy.is_shutdown():
        while not(cnt.alg.takeoff_finished):
            #Initialize takeoff location
            x_cmd = cnt.local_pos.x
            y_cmd = cnt.local_pos.y
            z_cmd = cnt.alg.takeoff_z

            cnt.updateSp(x_cmd, y_cmd, z_cmd)
            sp_pub.publish(cnt.sp)

            #Takeoff position reached.
            if(abs(z_cmd - cnt.local_pos.z) < 0.2 ):
                  cnt.alg.takeoff_finished = True
                  print('Take-off finished')
            
            rate.sleep()
            
        while(cnt.alg.takeoff_finished):
            x_cmd = 5.0
            y_cmd = 0.0
            z_cmd = 7.0
            cnt.updateSp(x_cmd, y_cmd, z_cmd)
            sp_pub.publish(cnt.sp)
            rate.sleep()

        
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

