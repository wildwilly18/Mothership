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
    rate = rospy.Rate(30.0)

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
    rospy.Subscriber('iris0/usb_cam/image_raw/compressed', CompressedImage, cnt.updateVisLoc, queue_size=3)

    # Subscribe to mothership drone's global position, update callback fnc
    rospy.Subscriber('uav1/mavros/global_position/global', NavSatFix, cnt.updateMothershipLoc)

    # Setpoint velocity publisher
    sp_vel = rospy.Publisher('uav0/mavros/setpoint_velocity/cmd_vel_unstamped', Twist)

    
    logFile = open(logName, 'w')
    
    printheader = cnt.logData('header')
    print(printheader)

    logFile.writelines(printheader)
    cnt.alg.algo_counter = 1000
    cnt.alg.visual_mode  = 1

    vel_cmd_x   = 0
    vel_cmd_y   = 0
    vel_cmd_z   = 0
    vel_cmd_yaw = 0

    cnt.updateVisualServoCMD(vel_cmd_x, vel_cmd_y, vel_cmd_z, vel_cmd_yaw)

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    for i in range(100):
        sp_vel.publish(cnt.sp_vel)
        rate.sleep()

    # activate OFFBOARD mode
    modes.setOffboardMode()

    print('OFFBOARD MODE SET...')
    print('ARMING')
    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    # ROS main loop
    while not rospy.is_shutdown():
        while not(cnt.alg.visual_mode):
            cnt.determineVisualAlg()
            cnt.determineEnterVisualMode()

            vel_cmd_x = 0
            vel_cmd_y = 0
            vel_cmd_z = -0.1
            vel_cmd_yaw = 0

            cnt.updateVisualServoCMD(vel_cmd_x, vel_cmd_y, vel_cmd_z, vel_cmd_yaw)

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

            if(cnt.mship_visible):
                vel_cmd_x   = -cnt.alg.x_vis_err
                vel_cmd_y   = -cnt.alg.y_vis_err
                vel_cmd_z   =  cnt.alg.z_vis_err
                vel_cmd_yaw =  cnt.alg.yaw_vis_err
            else:
                vel_cmd_x   =  0.0
                vel_cmd_y   =  0.0
                vel_cmd_z   =  -0.1
                vel_cmd_yaw =  0.0
                 
            cnt.updateVisualServoCMD(vel_cmd_x, vel_cmd_y, vel_cmd_z, vel_cmd_yaw)
            sp_vel.publish(cnt.sp_vel)
            print("Vel_CMD_X: " + str(vel_cmd_x) + " Vel_CMD_Y: " + str(vel_cmd_y) + " Vel_CMD_Z: " + str(vel_cmd_z) + " Vel_CMD_YAW: " + str(vel_cmd_yaw))

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