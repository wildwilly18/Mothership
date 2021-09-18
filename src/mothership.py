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

            #Localizing Algorithm info
            self.visual_mode        = 0 #When the quad is determine in position for step 2 this is true 
            self.vis_counter        = 0 #Track count of visual 
            self.vis_found_last     = 0 #Keep track if last check aruco was found. help
            self.vis_found_count    = 0 #Track consecutive vis found or not found for algorithm, weights single misses vs multiple frames of misses

            self.algo_counter       = 0 #Here tracks count of each frame where position error from A is < 0.2m for all dirs and Aruco tag 10 is identified
            self.mothership_vel     = 0 #Needed for calculating target for the quad
            self.mothership_heading = 0 #Heading of mothership
            self.pitch_2_match_vel  = 0 #Pitch required by quad to match mship vel

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

    def globalLoc(self, msg):
        self.sp_glob.latitude  = msg.latitude
        self.sp_glob.longitude = msg.longitude
        self.sp_glob.altitude  = msg.altitude

    def getTargetA(self, lat_target, lon_target, alt_target):
        self.A_lat = 0

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

    def determineInSafeZone(self, img):
        #Function here will use the mothership location and the Quadcopters external points with a mix of
        #visual location and gps loc to determine if the aircraft is in a state to stay/enter in visual servo mode.
        #See paper for explanation
        
        #Locate Aruco ID 10, If visualized count up that it is established.
        np_arr = np.fromstring(img.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        #Convert image to grey
        grey_im  = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(grey_im, self.alg.ARUCO_DICT, parameters=self.alg.ARUCO_PARAMS)

        if ids is not None:
            if 10 in ids:
                self.alg.vis_counter = self.alg.vis_counter + 1
            else:
                self.alg.vis_counter = self.alg.vis_counter - 1
                 
        #Check if rendesvouz location is established.
        x_err = abs(self.local_pos.x - self.alg.rs_target_x)
        y_err = abs(self.local_pos.y - self.alg.rs_target_y)
        z_err = abs(self.local_pos.z - self.alg.rs_target_z)

        #Check if errs < 0.2 m
        if(x_err < 0.2 and y_err < 0.2 and z_err < 0.2):
            self.alg.algo_counter = self.alg.algo_counter + 1
        else:
            self.alg.algo_counter = self.alg.algo_counter - 1

        #First check 
        if(self.alg.vis_counter > 10 and self.alg.algo_counter > 10):
            self.alg.safe = 1
        else:
            self.alg.safe = 0