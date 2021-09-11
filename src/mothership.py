#!/usr/bin/env python
# ROS python API
import roslib
import rospy
import cv2
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

        #intantiate an alg
        self.alg = self.Algorithm()

    class Algorithm:
        def __init__(self):     
            # For algorithm store which state of algorithm Quad is in. May need multiple aruco dictionaries for big, med, small Tags
            #Generated and this link https://chev.me/arucogen/
            #Image Algorithm info
            self.ARUCO_DICT   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_100) #Initialize the Aruco Dictionary that the controller will use.
            self.ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
            self.id_list      = [10]      #List of Aruco tags to use for localization. 
            self.id_loc_list  = [[-1000,-1000,-1000]] #Location of Aruco tags. Init to -1000 as an extraneous value.

            #Localizing Algorithm info
            self.safe         =  0 #When the quad is determine in position for step 2 this is true 

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

    def locateAruco(self, img):
        np_arr = np.fromstring(img.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        grey_im  = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(grey_im, self.alg.ARUCO_DICT, parameters=self.alg.ARUCO_PARAMS)
        cv2.imshow('cv_img', grey_im)
        print ids
        cv2.waitKey(2)

    def determineSafeZone(self):
        #Function here will use the mothership location and the Quadcopters external points with a mix of
        #visual location and gps loc to determine if the aircraft is ready to visual servo in.
        if(insideSafeZone):
            self.alg.safe = 1

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