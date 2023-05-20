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
    rospy.Subscriber('iris0/usb_cam/image_raw/compressed', CompressedImage, cnt.locateAruco, queue_size=5)

    # Subscribe to mothership drone's global position, update callback fnc
    rospy.Subscriber('uav1/mavros/global_position/global', NavSatFix, cnt.updateMothershipLoc)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('uav0/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Setpoint velocity publisher
    sp_vel = rospy.Publisher('uav0/mavros/setpoint_velocity/cmd_vel_unstamped', Twist)

    modes.setPositionMode()
    rate.sleep()

    print('Initializing Lat Lon')
    cnt.initLatLon()

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
    cnt.alg.takeoff_finished = 0
    # ROS main loop
    while not rospy.is_shutdown():
        #Takeoff State
        while not(cnt.alg.takeoff_finished):
            #Initialize takeoff location
            x_cmd = cnt.local_pos.x
            y_cmd = cnt.local_pos.y
            z_cmd = cnt.alg.takeoff_z

            cnt.updateSp(x_cmd, y_cmd, z_cmd)
            sp_pub.publish(cnt.sp)

            #Takeoff position reached.
            if(abs(z_cmd - cnt.local_pos.z) < 0.2 ):
                  cnt.alg.rendesvous_mode = 1
                  cnt.alg.takeoff_finished = True
                  print('Take-off finished')
            
            rate.sleep()

        #Rendesvous State
        while(cnt.alg.takeoff_finished and cnt.mship_located and not cnt.alg.at_rendesvous):
            cnt.updateRendesvousLoc()

            x_cmd = cnt.alg.rs_target_x
            y_cmd = cnt.alg.rs_target_y
            z_cmd = cnt.alg.rs_target_z

            cnt.determineAtRendesvous()

            cnt.updateSp(x_cmd, y_cmd, z_cmd)
            sp_pub.publish(cnt.sp)
            rate.sleep()
            if(cnt.alg.at_rendesvous):
                 print('Begin Search for Tag')
        
        #Visual Tag Identification State
        while(not cnt.alg.visual_mode):
            cnt.determineVisualAlg()
            cnt.determineEnterVisualMode()
            cnt.updateRendesvousLoc()
            cnt.determineAtRendesvous()

            if(not cnt.mship_visible and not cnt.alg.searchPointUpdated):
                print('Updating Search Point')
                a = cnt.alg.search_count % 8
                cnt.updateSearchPoint(cnt.alg.search_x_array[a], cnt.alg.search_y_array[a])
                cnt.clearVisOffset()
                cnt.updateRendesvousLoc()

            if(not cnt.mship_visible and cnt.alg.at_rendesvous):
                cnt.alg.searchPointUpdated = False
                cnt.alg.at_rendesvous = False
                cnt.alg.search_count = cnt.alg.search_count + 1
                cnt.clearAlgoCounter()
                print("Search Count : " + str(cnt.alg.search_count))

            if(cnt.mship_visible and cnt.alg.at_rendesvous and not cnt.alg.markerFound):
                #cnt.clearSearchPointForVisOffset()
                cnt.checkMarkerSetMarkerRendesvouzPoint()
                cnt.updateRendesvousLoc()
                cnt.alg.markerFound = True
                print("Marker Found")

            x_cmd = cnt.alg.rs_target_x
            y_cmd = cnt.alg.rs_target_y
            z_cmd = cnt.alg.rs_target_z

            cnt.updateSp(x_cmd, y_cmd, z_cmd)
            sp_pub.publish(cnt.sp)
            rate.sleep()

        #State for visual mode, here we command velocities and slowly move up to the marker until within range
        while(cnt.alg.visual_mode):
            cnt.determineVisualAlg()
            cnt.determineExitVisualMode()
            cnt.checkVisAppInRadius()
            cnt.updateVisualDist()

            vel_cmd_x   = -cnt.alg.x_vis_err
            vel_cmd_y   = -cnt.alg.y_vis_err
            vel_cmd_z   = cnt.alg.z_vis_err - cnt.alg.vis_app_dist
            vel_cmd_yaw = cnt.alg.yaw_vis_err

            print('Z Velocity Command: ' + str(vel_cmd_z) + ' Vis App Dist: ' + str(cnt.alg.vis_app_dist))
            cnt.updateVisualServoCMD(vel_cmd_x, vel_cmd_y, vel_cmd_z, -vel_cmd_yaw)
            sp_vel.publish(cnt.sp_vel)
            rate.sleep()

        
        
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

