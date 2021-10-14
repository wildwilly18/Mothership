#!/usr/bin/env python
# ROS python API
import roslib
import rospy
import cv2
import math
import numpy as np

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Twist
from sensor_msgs.msg   import NavSatFix
from std_msgs.msg      import Float64
from cv_bridge         import CvBridge
from sensor_msgs.msg   import CompressedImage

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

#Gazebo messasges
from gazebo_msgs.srv import GetModelState

class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()

        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # Instantiate a "mavros/setpoint_raw/global" message
        self.sp_glob = GlobalPositionTarget()
        self.sp_glob.type_mask = int('010111111000', 2)
        self.sp_glob.coordinate_frame = 6 #FRAME_GLOBAL_INT
        self.sp_glob.latitude  = 0
        self.sp_glob.longitude = 0
        self.sp_glob.altitude  = 0
        self.sp_glob.yaw       = 0
        
        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 10.0
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP

        #Match the velocity of the flying craft
        #self.sp.velocity.x = #x portion of velocity vector.
        #self.sp.velocity.y = #y portion of velocity vector.

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 1.0)

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.yaw        = 4.0

        #initiate a Mothership
        self.mShip = self.Mothership()
        #intantiate an alg
        self.alg   = self.Algorithm()

    class Algorithm:
        def __init__(self):     
            # For algorithm store which state of algorithm Quad is in. May need multiple aruco dictionaries for big, med, small Tags
            #Generated and this link https://chev.me/arucogen/
            #Image Algorithm info
            self.ARUCO_DICT    = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_100) #Initialize the Aruco Dictionary that the controller will use.
            self.ARUCO_PARAMS  = cv2.aruco.DetectorParameters_create()
            self.id_list       = [10]      #List of Aruco tags to use for localization. 
            self.id_loc_list   = [[-1000,-1000,-1000]] #Location of Aruco tags. Init to -1000 as an extraneous value.
            self.camera_matrix = np.array([[277.191356, 0, 320.5],[0, 277.191356, 240.5], [0, 0, 1]]) #Camera matrix for simulated camera. From fpv_cam.sdf
            self.camera_dist   = np.array([0, 0, 0, 0]) #Distortion Coefficients for the simlated camera. set to 0 in sim. From fpv_cam.sdf
            self.img           = None

            #Localizing Algorithm info
            self.visual_mode            =    0 #When the quad is determine in position for step 2 this is true
            self.visual_first_encounter =    0 #False until first time a marker is detected. 
            self.vis_counter            =    0 #Track count of visual 
            self.vis_last               =    0 #Keep track if last check aruco was found.
            self.vis_consecutive        =    0 #Track consecutive vis found or not found for algorithm, weights single misses vs multiple frames of misses
            self.vis_counter_max_min    = 1000 #Saturation value for the visual counter

            #Rendesvouz algorithm trackers
            self.rendesvous_mode      =    0 #Set the rendesvous mode on startup. 
            self.algo_counter         =    0 #Here tracks count of each frame where position error from A is < specified value for all dirs
            self.algo_last            =    0 #Track if previous frame aircraft was in the range
            self.algo_consecutive     =    0 #Track consecutive frames of within error range
            self.algo_counter_sat     = 1000 #Saturation value for position
            self.algorithm_threshold  =  750 #Value for algorithm thresholds
            self.mothership_vel       = 0    #Needed for calculating target for the quad
            self.mothership_heading   = 0    #Heading of mothership
            self.pitch_2_match_vel    = 0    #Pitch required by quad to match mship vel
            self.rendesvouz_int       = 0    #Integrate the error for Rendesvouz command point. 
            self.rendesvouz_int_gain  = 1.5  #Gain for the rendesvouz integrator error.
            self.error_vec_last       = 0    #Error from previous for integrator
            self.x_error_int          = 0    #x integrator value
            self.y_error_int          = 0    #y integrator value

            #Rendeszous target, 2m distance from the ship. Position behind and below to match quads estimated pitch for speed.
            self.rs_target_x = -2 * math.cos(self.pitch_2_match_vel)
            self.rs_target_y =  0
            self.rs_target_z =  2 * math.sin(self.pitch_2_match_vel)
            
            #Visual Servo X, Y & Z positions, start at 2 m dist off the mship. Is a func of mship speed and pithc
            #Below assumption is moving in x dir only
            self.vs_target_x = -2 * math.cos(self.pitch_2_match_vel)
            self.vs_target_y =  0 #eventually will 
            self.vs_target_z =  2 * math.sin(self.pitch_2_match_vel)
    #Class to get the information from the Mothership in the sim.

    class Mothership:
        def __init__(self):
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            mShip_coordinates = model_coordinates("mothership", "")

            self.x = mShip_coordinates.pose.position.x
            self.y = mShip_coordinates.pose.position.y
            self.z = mShip_coordinates.pose.position.z

            print str( str('Mothership found at, X: ') + str(self.x)+ str(' Y: ') + str(self.y)+ str(' Z: ') + str(self.z))

        def get_sim_location(self):
            #Just get the values from the ros connection to get the model state to use
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            mShip_coordinates = model_coordinates("mothership", "")

            #Update the locations to the Mothership object
            self.x = mShip_coordinates.pose.position.x
            self.y = mShip_coordinates.pose.position.y
            self.z = mShip_coordinates.pose.position.z

        def get_sim_world_location(self):
            #This function will need to be updated to transfer from sim location to World Location
            #Just get the values from the ros connection to get the model state to use
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            mShip_coordinates = model_coordinates("mothership", "")

            #Update the locations to the Mothership object
            self.x = mShip_coordinates.pose.position.x
            self.y = mShip_coordinates.pose.position.y
            self.z = mShip_coordinates.pose.position.z

	# Callbacks
    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    # Drone heading
    def updateHDG(self, msg):
        self.heading = msg.data

    ## Update setpoint message
    def updateSp(self, x_des, y_des, z_des):
        self.sp.position.x = x_des
        self.sp.position.y = y_des
        self.sp.position.z = z_des
        self.sp.yaw        = 2.0

    def globalLoc(self, msg):
        self.sp_glob.latitude  = msg.latitude
        self.sp_glob.longitude = msg.longitude
        self.sp_glob.altitude  = msg.altitude

    def getTargetA(self, lat_target, lon_target, alt_target):
        self.A_lat = 0

    def updateRendesvousLoc(self):
        self.mShip.get_sim_location()

        self.alg.rs_target_x = self.mShip.x + (2 * math.sin(0)) #Eventually will be heading
        self.alg.rs_target_y = self.mShip.y + (0) #Eventually will be heading 
        self.alg.rs_target_z = self.mShip.z - (2 * math.cos(self.alg.pitch_2_match_vel))

        #Check error and integrate it.
        error_vec = math.sqrt(((self.local_pos.x - self.alg.rs_target_x) * (self.local_pos.x - self.alg.rs_target_x)) + ((self.local_pos.y - self.alg.rs_target_y) * (self.local_pos.y - self.alg.rs_target_y)))

        #Integrate error
        rendesvouz_int = self.alg.rendesvouz_int + (0.5 * (error_vec + self.alg.error_vec_last))
        self.alg.rendesvouz_int = rendesvouz_int
        #Store error for last error
        self.alg.error_vec_last = error_vec

        x_error_int = error_vec * math.cos(0) * self.alg.rendesvouz_int_gain
        y_error_int = error_vec * math.sin(0) * self.alg.rendesvouz_int_gain

        #Add integrator to target.
        print x_error_int
        self.alg.rs_target_x = self.mShip.x + (2 * math.sin(0)) + x_error_int #Eventually will be heading
        self.alg.rs_target_y = self.mShip.y  #Eventually will be heading 
        self.alg.rs_target_z = self.mShip.z - (2 * math.cos(self.alg.pitch_2_match_vel))        

    def updateVisLoc(self, img):
        self.mShip.get_sim_location()

        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.alg.ARUCO_DICT, parameters=self.alg.ARUCO_PARAMS)

        if ids is not None:
            for id in ids:
                #Set the length of the ID detected.
                if(id[0] == 10):
                    aruco_len = 0.1
                    #Get the rotation vec and translation vec of the camera to the aruco I believe. can use this to control the quad.
                    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners[0], aruco_len, self.alg.camera_matrix, self.alg.camera_dist)
                    #Simplify here. Going directly below. Will need to figure out how to calc err from a spot.
                    self.alg.vs_target_x = self.mShip.x - (tvecs[0][0][0] * 0.1)
                    self.alg.vs_target_y = self.mShip.y - (tvecs[0][0][1] * 0.1)
                    self.alg.vs_target_z = self.mShip.z - (tvecs[0][0][2] * 0.1) - .5

                    print str(" Location: X: " + str(self.mShip.x)+ " Y: " + str(self.mShip.y) + " Z: " +str(self.mShip.z))
                    #print str("Aruco Loc: X: " + str(tvecs[0][0][0])+ " Y: " + str(tvecs[0][0][1]) + " Z: " +str(tvecs[0][0][2]))
                    print str("  Des Loc: X: " + str(self.alg.vs_target_x)+ " Y: " + str(self.alg.vs_target_y) + " Z: " +str(self.alg.vs_target_z))
                    
    def imgCallback(self, img):
        np_arr = np.fromstring(img.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        #Convert image to grey
        grey_im = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)

        #Save this image in our object
        self.alg.img = grey_im

    def locateArucoID(self, img, id):
        np_arr = np.fromstring(img.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        #Convert image to grey
        grey_im  = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(grey_im, self.alg.ARUCO_DICT, parameters=self.alg.ARUCO_PARAMS)

        if ids is not None:
            for id in ids:
                #Set the length of the ID detected.
                if(id[0] == 10):
                    #Get the rotation vec and translation vec of the camera to the aruco I believe. can use this to control the quad.
                    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners[cnt], aruco_len, self.alg.camera_matrix, self.alg.camera_dist)

        #Printing for debug purposes
        loc_string = str('yaw: ') + str(rvecs[0][0][2]) + str(' z-dist: ') + str(tvecs[0][0][2])
        print loc_string
        cnt = cnt + 1
        err = [rvecs[0][0][2], tvecs[0][0][2]]
        
        return err
        cv2.imshow('cv_img', grey_im)
        cv2.waitKey(2)

    def locateAruco(self, img):
        np_arr = np.fromstring(img.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        #Convert image to grey
        grey_im  = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(grey_im, self.alg.ARUCO_DICT, parameters=self.alg.ARUCO_PARAMS)

        if ids is not None:
            cnt = 0

            for id in ids:
                #Set the length of the ID detected.
                if(id[0] == 10):
                    aruco_len = 0.1
                if(id[0] == 20):
                    aruco_len = 0.25

                #Get the rotation vec and translation vec of the camera to the aruco I believe. can use this to control the quad.
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners[cnt], aruco_len, self.alg.camera_matrix, self.alg.camera_dist)

                #Printing for debug purposes
                loc_string = str('yaw: ') + str(rvecs[0][0][2]) + str(' z-dist: ') + str(tvecs[0][0][2])
                print loc_string
                cnt = cnt + 1
                

        cv2.imshow('cv_img', grey_im)
        cv2.waitKey(2)

    def determineAtRendesvous(self):
        #At Rendesvouz if Quadrotor is withing 0.5m error in x,y,z count up.
        x_err = abs(self.local_pos.x - self.alg.rs_target_x)
        y_err = abs(self.local_pos.y - self.alg.rs_target_y)
        z_err = abs(self.local_pos.z - self.alg.rs_target_z)

        print str( "X err: " + str(x_err) + " Y err: " + str(y_err) + " Z err: " + str(z_err))
        if(x_err < 0.7 and y_err < 0.7 and z_err < 0.7):
            if self.alg.algo_last is 1:
                #If isn't saturated add to counter. If saturated Pass
                if (self.alg.algo_counter < self.alg.algo_counter_sat):
                    self.alg.algo_counter = self.alg.algo_counter + (1 * self.alg.algo_consecutive)
                    self.alg.algo_consecutive = self.alg.algo_consecutive + 1

                    #Check if it becomes saturated
                    if (self.alg.algo_counter > self.alg.algo_counter_sat):
                        self.alg.algo_counter = self.alg.algo_counter_sat
                else:
                    pass

            else:
                self.alg.algo_counter     = self.alg.algo_counter + 1
                self.alg.algo_consecutive = 1
                self.alg.algo_last        = 1

        else:
            if self.alg.algo_last is 0:
                if(self.alg.algo_counter > -self.alg.algo_counter_sat):
                    self.alg.algo_counter     = self.alg.algo_counter - (1 * self.alg.algo_consecutive)
                    self.alg.algo_consecutive = self.alg.algo_consecutive + 1

                    if(self.alg.algo_counter < -self.alg.algo_counter_sat):
                        self.alg.algo_counter = -self.alg.algo_counter_sat
                else:
                    pass
            else:
                self.alg.algo_counter     = self.alg.algo_counter - 1
                self.alg.algo_consecutive = 1
                self.alg.algo_last        = 0

    def determineVisualAlg(self, img):
         #Gets image from our object
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.alg.ARUCO_DICT, parameters=self.alg.ARUCO_PARAMS)

        #Check here if ID 10 is located. Build our visual saturation off of this.
        if ids is not None:
            if 10 in ids:
                self.alg.visual_first_encounter = 1
                #Check if aruco with ID 10 was found the last frame
                if self.alg.vis_last is 1:
                    #If counter isn't saturated add to counter using funciton and increment counter. If Saturated, Pass
                    if (self.alg.vis_counter < self.alg.vis_counter_max_min):
                        self.alg.vis_counter = self.alg.vis_counter + (1 * self.alg.vis_consecutive)
                        self.alg.vis_consecutive = self.alg.vis_consecutive + 1

                        #If counter becomes greater than the saturation value then set it to max/min value.
                        if (self.alg.vis_counter > self.alg.vis_counter_max_min):
                            self.alg.vis_counter = self.alg.vis_counter_max_min
                    else:
                        pass

                #If first time seeing the ID after  intitialize last/consecutive variables. Don't reset counter 
                else:
                    self.alg.vis_last        = 1
                    self.alg.vis_consecutive = 1
                    self.alg.vis_counter = self.alg.vis_counter + (1 * self.alg.vis_consecutive)

            #Doing the inverse if not found.
            else:
                if self.alg.vis_last is 0:
                    #If counter isn't saturated add to counter using function and increment counter. If Saturated pass.
                    if(self.alg.vis_counter > -self.alg.vis_counter_max_min):
                        self.alg.vis_counter = self.alg.vis_counter - (1 * self.alg.vis_consecutive)
                        self.alg.vis_consecutive = self.alg.vis_consecutive + 1

                        #If counter becomes less than the saturation max/min value then set to max/min value.
                        if(self.alg.vis_counter < -self.alg.vis_counter_max_min):
                            self.alg.vis_counter = -self.alg.vis_counter_max_min
                    else:
                        pass
                else:
                    self.alg.vis_last        =  0
                    self.alg.vis_consecutive =  1
                    self.alg.vis_counter = self.alg.vis_counter - (1 * self.alg.vis_consecutive)

                    #Doing the inverse if not found.
        #If we have encountered the tag for the first time and we no longer see the tag remove confidence in visual
        elif(self.alg.visual_first_encounter == 1):
            if self.alg.vis_last is 0:
                #If counter isn't saturated add to counter using function and increment counter. If Saturated pass.
                if(self.alg.vis_counter > -self.alg.vis_counter_max_min):
                    self.alg.vis_counter = self.alg.vis_counter - (1 * self.alg.vis_consecutive)
                    self.alg.vis_consecutive = self.alg.vis_consecutive + 1

                    #If counter becomes less than the saturation max/min value then set to max/min value.
                    if(self.alg.vis_counter < -self.alg.vis_counter_max_min):
                        self.alg.vis_counter = -self.alg.vis_counter_max_min
                else:
                    pass
            else:
                self.alg.vis_last        =  0
                self.alg.vis_consecutive =  1
                self.alg.vis_counter = self.alg.vis_counter - (1 * self.alg.vis_consecutive)

    def determineEnterVisualMode(self):
        #Check the counters to see if at algorithm location and visual target identified
        if (self.alg.vis_counter > self.alg.algorithm_threshold and self.alg.algo_counter > self.alg.algorithm_threshold):
            #Set visual mode true if thresholds met. 
            self.alg.visual_mode = 1
    
    def determineExitVisualMode(self):
        #Algorithm counter is not used to determine exit of visual mode
        if(self.alg.vis_counter < self.alg.algorithm_threshold):
            #Exit visual mode and Reset first encounter of tag and reset the counter to 0
            self.alg.visual_mode            = 0
            self.alg.visual_first_encounter = 0
            self.alg.vis_counter            = 0
