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
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # activate OFFBOARD mode
    modes.setOffboardMode()

    # ROS main loop
    while not rospy.is_shutdown():
        #Start of the algorithm here.
        #Calculate Rendesvous location
        cnt.updateRendesvousLoc()

        #Determine if the Aircraft is at Rendesvous Location.
        cnt.determineAtRendesvous()
        #Look for an ID and start tracking image confidence
        cnt.determineVisualAlg(cnt.alg.img)

        #Update the drone with the set point target
        cnt.updateRendesvousLoc()
        cnt.updateSp(cnt.alg.rs_target_x, cnt.alg.rs_target_y, cnt.alg.rs_target_z)
        
        #Update the set point of the quad rotor.
        sp_pub.publish(cnt.sp)
        #Check if visual mode can be entered. If so function will update object with visual mode true
        cnt.determineEnterVisualMode()
        print str("Visual Counter: " + str(cnt.alg.vis_counter) + " lg Counter: " + str(cnt.alg.algo_counter))
        
        #If object can enter visual mode it will enter this loop. This loop only uses visual for confidence.
        while cnt.alg.visual_mode:
            
            print str("Visual Counter: " + str(cnt.alg.vis_counter))
            #Locate the Aruco Marker and update 
            cnt.determineVisualAlg(cnt.alg.img)

            #Check if we exit visual mode
            cnt.determineExitVisualMode()

            #If still in visual mode
            if cnt.alg.visual_mode:
                cnt.updateVisLoc(cnt.alg.img)
                cnt.updateSp(cnt.alg.vs_target_x, cnt.alg.vs_target_y, cnt.alg.vs_target_z)
                
                #Update the setpoint of the quadrotor based on the image translations.
                sp_pub.publish(cnt.sp)

            else:
                pass #Pass and will exit since visual mode no longer is true and return to Rendesvouz mode. 

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
