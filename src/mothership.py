#!/usr/bin/env python
# ROS python API
import roslib
import rospy
import cv2
import math
import numpy as np
import time
import tf.transformations
import pymap3d as pm
import platform

# 3D point & Stamped Pose msgs
from geometry_msgs.msg       import Point, PoseStamped, Twist
from sensor_msgs.msg         import NavSatFix, Imu
from std_msgs.msg            import Float64
from cv_bridge               import CvBridge
from sensor_msgs.msg         import CompressedImage
from scipy.spatial.transform import Rotation as R

# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('uav0/mavros/cmd/takeoff')    
        try:
            takeoffService = rospy.ServiceProxy('uav0/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = 3)
        except rospy.ServiceException as e:
            print("Service takeoff call failed%s"%e)

    def setArm(self):
        rospy.wait_for_service('uav0/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('uav0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s"%e)

    def setDisarm(self):
        rospy.wait_for_service('uav0/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('uav0/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s"%e)

    def setStabilizedMode(self):
        rospy.wait_for_service('uav0/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Stabilized Mode could not be set."%e)

    def setOffboardMode(self):
        rospy.wait_for_service('uav0/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set."%e)

    def setAltitudeMode(self):
        rospy.wait_for_service('uav0/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Altitude Mode could not be set."%e)

    def setPositionMode(self):
        rospy.wait_for_service('uav0/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Position Mode could not be set."%e)

    def setAutoLandMode(self):
        rospy.wait_for_service('uav0/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('uav0/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Autoland Mode could not be set."%e)

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()

        # set the flag to use position setpoints and yaw angle
        #self.sp.type_mask = int('010111111000', 2)
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

        # Instantiate a "mavros/setpoint_velocity/cmd_vel_unstamped"
        self.sp_vel = Twist()
        self.sp_vel.linear.x = 0.0
        self.sp_vel.linear.y = 0.0
        self.sp_vel.linear.z = 0.0
        #Angular Commands.. we only will cmd z need all though.
        self.sp_vel.angular.x = 0.0
        self.sp_vel.angular.y = 0.0
        self.sp_vel.angular.z = 0.0

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 10.0
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 1.0)

        self.yaw   = 0.0
        self.pitch = 0.0
        self.roll  = 0.0

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.yaw        = 0.0
        self.sp.yaw_rate   = 0.0

        # Global2World Reference frame to convert mothership into chase ship local coordinates.
        self.lat0 = 0.0
        self.lon0 = 0.0
        self.alt0 = 0.0

        # Also need to save xyz of origin or else conversions wont work.
        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = 0.0

        #Flag to set if global origin set
        self.globalOrigin_Set = False

        # Mothership global_loc
        self.mship_lat = 0.0
        self.mship_lon = 0.0
        self.mship_h   = 0.0

        # Mothership local_loc
        self.mship_x   = 0.0
        self.mship_y   = 0.0
        self.mship_z   = 0.0

        self.mship_x_err             = 0.0
        self.mship_y_err             = 0.0
        self.mship_z_err             = 0.0
        self.mship_visible           = 0
        self.mship_not_visible_count = 0
        self.mship_debounce          = 20 #require 20 frames of not identifying an aruco to say we cant see it.

        self.mship_x_avg   = 0.0
        self.mship_y_avg   = 0.0
        self.mship_z_avg   = 0.0


        #Flag for when the mothership is located. This could get extremely complex in a real application but for now keeping it pretty simple
        self.mship_located = False

        #intantiate an alg class
        self.alg   = self.Algorithm()

    class Algorithm:
        def __init__(self):     
            # For algorithm store which state of algorithm Quad is in. May need multiple aruco dictionaries for big, med, small Tags
            #Generated and this link https://chev.me/arucogen/
            #Image Algorithm info
            self.ARUCO_DICT    = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_100) #Initialize the Aruco Dictionary that the controller will use.
            self.ARUCO_PARAMS  = cv2.aruco.DetectorParameters_create()

            my_system = platform.uname()

            if(my_system.node == 'wilsons-PC'):
                fx = 5.522067755300895e+02
                fy = 7.449299628154979e+02
                cx = 3.479833806300082e+02
                cy = 2.925025107082283e+02
                k1 = -0.425968726364035
                k2 =  0.176325076893194
                #fx = 277.191356
                #fy = 277.191356
                #cx = 159.5
                #cy = 119.5
                #k1 = 0
                #k2 = 0
                #hfov = 1.349
            else:
                #Camera intrinsic paramaters determined from matlab camera calibration...
                #Set if we know we aren't simming.
                fx = 5.522067755300895e+02
                fy = 7.449299628154979e+02
                cx = 3.479833806300082e+02
                cy = 2.925025107082283e+02
                k1 = -0.425968726364035
                k2 =  0.176325076893194
            
            self.camera_matrix = np.array([[fx, 0, cx],[0, fy, cy], [0, 0, 1]]) #Camera matrix for simulated camera. From fpv_cam.sdf
            self.camera_dist   = np.array([k1, k2, 0, 0]) #Distortion Coefficients for the simlated camera. set to 0 in sim. From fpv_cam.sdf
            self.img           = None

            #self.camera_matrix = np.array([[277.191356, 0, 159.5],[0, 277.191356, 119.5], [0, 0, 1]]) #Camera matrix for simulated camera. From fpv_cam.sdf
            #self.camera_dist   = np.array([0, 0, 0, 0]) #Distortion Coefficients for the simlated camera. set to 0 in sim. From fpv_cam.sdf
            #self.img           = None

            #Localizing Algorithm info
            self.visual_mode            =     0 #When the quad is determine in position for step 2 this is true
            self.visual_first_encounter =     0 #False until first time a marker is detected. 
            self.vis_counter            =     0 #Track count of visual 
            self.vis_last               =     0 #Keep track if last check aruco was found.
            self.vis_consecutive        =     0 #Track consecutive vis found or not found for algorithm, weights single misses vs multiple frames of misses
            self.vis_counter_max_min    =  1000 #Saturation value for the visual counter
            self.vis_P                  =   0.7 #Proportional gain for our Visual Controller, may need independant X and Y

            #Takeoff values
            self.takeoff_finished      = 0
            self.takeoff_x             = 0.0
            self.takeoff_y             = 0.0
            self.takeoff_z             = 4.0

            #Rendesvouz algorithm trackers
            self.rendesvous_mode      =      0    #Set the rendesvous mode on startup. 
            self.at_rendesvous        =      0    #Set at the rendesvouz location.
            self.algo_counter         =      0    #Here tracks count of each frame where position error from A is < specified value for all dirs
            self.algo_last            =      0    #Track if previous frame aircraft was in the range
            self.algo_consecutive     =      0    #Track consecutive frames of within error range
            self.algo_counter_sat     =   1000    #Saturation value for position
            self.algorithm_threshold  =    750    #Value for algorithm thresholds
            self.rendesvouz_dist      =    2.5    #distance in M that the rendesvouz point will be
            self.quad_radius          =    0.1    #Uncertainty sphere radius of the quadrotor
            self.safe_radius          =    1.2    #Store the safe radius that is calculated for the visual algorithm
            self.err_mag              =    0.0    #Err mag calculated for the visual algorithm
            self.x_vis_offset         =    0.0    #Offset based on a visual id of marker for x
            self.y_vis_offset         =    0.0    #Offset based on a visual id of marker for y

            #Previous Set Point Value for storage
            self.x_setpoint_prev      =    0.0
            self.y_setpoint_prev      =    0.0
            self.z_setpoint_prev      =    0.0
            self.mode_fade_increment  =    0.0     #Fade value to go from rendesvouz to visual error tracking

            #Visual Approach Mode trackers
            self.vis_app_dist = self.rendesvouz_dist #Initialize our start with where we rendesvouz too
            self.vis_app_counter           =       0    #Counter to track that we are inside the allowed
            self.vis_app_last              =       0    
            self.vis_app_consecutive       =       0
            self.vis_app_threshold         =     750 #Threshold to turn on or off distance subtraction.
            self.vis_app_counter_min_max   =    1250
            self.vis_app_dist_sub_rate     =   .0001 #Shows how many meters will be removed per frame. runs @~100Hz. this is ~1mm per second. about 1 minute to go 1m
            self.quad_safe                 =       0 #Store if the quad is in the safe zone for visual servo
            self.x_vis_err                 =     0.0
            self.y_vis_err                 =     0.0 
            self.z_vis_err                 =     0.0
            self.yaw_vis_err               =     0.0
            self.vis_target_dist           =    0.25 #How many meters the quad will target to be from the target

            #Rendezvous target, 2m distance from the ship. Position behind and below to match quads estimated pitch for speed.
            self.rs_target_x = 0.0
            self.rs_target_y = 0.0
            self.rs_target_z = self.rendesvouz_dist
            
            #Visual Servo X, Y & Z positions, start at 2 m dist off the mship. Is a func of mship speed and pithc
            #Below assumption is moving in x dir only
            self.vs_target_x =  0.0
            self.vs_target_y =  0.0
            self.vs_target_z =  self.rendesvouz_dist

            #Save mixed target values as to not overwrite Visual or Rendezvous targets
            self.fade_target_x = 0.0
            self.fade_target_y = 0.0
            self.fade_target_z = 0.0 

	# Callbacks
    ## Mothership location callback
    def updateMothershipLoc(self, msg):
        if self.globalOrigin_Set:
            self.mship_lat = msg.latitude
            self.mship_lon = msg.longitude
            self.mship_h   = msg.altitude

            #Take the input lat lon h and lat0 lon0 h0 for the chase to get local coordinate conversion
            (self.mship_x_err, self.mship_y_err, self.mship_z_err) = pm.geodetic2enu(self.mship_lat, self.mship_lon, self.mship_h, self.sp_glob.latitude, self.sp_glob.longitude, self.sp_glob.altitude)

        if(self.mship_located == False):
            self.mship_located = True
            print(str('Mothership found'))

    def orientation(self, msg):
        orientation_q = msg.orientation

        #print orientation_q
    
        angles = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        #print angles

    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

        #Get Quart and convert to Euler
        orientation_q = msg.pose.orientation
        angles = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        #print angles
        self.roll  = angles[0]
        self.pitch = angles[1]
        self.yaw   = angles[2]


    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def initLatLon(self):
        if not self.globalOrigin_Set:
            self.x0 = self.local_pos.x
            self.y0 = self.local_pos.y
            self.z0 = self.local_pos.z
            #Upon startup of controller set the initial latlon to be used to get xyz of mothership in future.
            (self.lat0, self.lon0, self.alt0) = pm.enu2geodetic(self.x0, self.y0, self.z0, self.sp_glob.latitude, self.sp_glob.longitude, self.sp_glob.altitude)

            print('Drone Initialized at Lat:' + str(self.lat0) +' Lon:' + str(self.lon0) + ' Alt:' + str(self.alt0))
            print('Sim Origin at: ' + str(self.x0) + ' Y: ' + str(self.y0) + ' Z: ' + str(self.z0))
            self.globalOrigin_Set = True
        else:
            print("Whoops! Global origin is already set... Warning!!! Warning!!!")
    
    # Drone heading
    def updateHDG(self, msg):
        self.heading = msg.data

    ## Update setpoint message
    def updateSp(self, x_des, y_des, z_des, yaw=0):
        self.sp.position.x = x_des
        self.sp.position.y = y_des
        self.sp.position.z = z_des
        self.sp.yaw        = yaw

    def updateVel(self, x_vel, y_vel, z_vel, yaw_rate):
        self.sp_vel.linear.x = x_vel
        self.sp_vel.linear.y = y_vel
        self.sp_vel.linear.z = z_vel
        self.sp_vel.angular.z = yaw_rate

    def globalLoc(self, msg):
        self.sp_glob.latitude  = msg.latitude
        self.sp_glob.longitude = msg.longitude
        self.sp_glob.altitude  = msg.altitude

    def updateRendesvousLoc(self):
        self.alg.rs_target_x = self.local_pos.x + self.mship_x_err + self.alg.x_vis_offset
        self.alg.rs_target_y = self.local_pos.y + self.mship_y_err + self.alg.y_vis_offset
        self.alg.rs_target_z = self.local_pos.z + self.mship_z_err - self.alg.rendesvouz_dist  

        #Trying to Debug
        #print('Going to X: ' + str(self.alg.rs_target_x) + ' Y: ' + str(self.alg.rs_target_y) + ' Z: ' + str(self.alg.rs_target_z))     
    def updateVisualServoCMD(self, x_error, y_error, z_error, yaw_error):
        self.sp_vel.linear.x = self.saturateValue((x_error*0.5), -0.7, 0.7)
        self.sp_vel.linear.y = self.saturateValue((y_error*0.5), -0.7, 0.7)
        self.sp_vel.linear.z = self.saturateValue((z_error*0.8), -0.5, 0.5)
        self.sp_vel.angular.z = self.saturateValue((yaw_error*0.002), -0.5, 0.5)

    def updateVisErr(self, cam2aruco):
        d = self.alg.vis_app_dist
        
        x_vis = cam2aruco[0]
        y_vis = cam2aruco[1]
        z_vis = cam2aruco[2]

        #Calculate x y z error of the quad relative to the target.
        self.alg.x_vis_err =  x_vis 
        self.alg.y_vis_err =  y_vis 
        self.alg.z_vis_err =  d - z_vis

        #print str('x_vis_err:' + str(self.alg.x_vis_err))

    def checkVisAppInRadius(self):
        #Calculate (x,y,z) location of vis guidance center
        d = self.alg.vis_app_dist
        quad_rad = self.alg.quad_radius

        #Calculate the safe zone radius
        safe_radius = 0.5 * (d**2 / self.alg.rendesvouz_dist)  #R = 0.5* (d^2 / d_not)

        #If close to quad use other portion of the system of equations.
        if(safe_radius < (2 * quad_rad)):
            safe_radius = 2 * quad_rad

        err_mag = math.sqrt((self.alg.x_vis_err ** 2) + (self.alg.y_vis_err ** 2) + ((self.alg.z_vis_err - self.alg.vis_app_dist) ** 2))

        #print str("Error Mag: " + str(err_mag) + " safe_radius: " + str(safe_radius))
        self.alg.safe_radius = safe_radius
        self.alg.err_mag     = err_mag

        if(err_mag + quad_rad < safe_radius):
            self.alg.quad_safe = 1
        else:
            self.alg.quad_safe = 0

    def updateVisualDist(self):
        #Function here will store the confidence of the quads position in approach. It will also calculate D to move the quad closer to the target
        if(self.alg.quad_safe):
            if self.alg.vis_app_last == 1:
                #If isn't saturated add to counter. If saturated Pass
                if (self.alg.vis_app_counter < self.alg.vis_app_counter_min_max):
                    self.alg.vis_app_counter     = self.alg.vis_app_counter + (1 * self.alg.vis_app_consecutive)
                    self.alg.vis_app_consecutive = self.alg.vis_app_consecutive + 1

                    #Check if it becomes saturated
                    if (self.alg.vis_app_counter > self.alg.vis_app_counter_min_max):
                        self.alg.vis_app_counter = self.alg.vis_app_counter_min_max
                else:
                    pass

            else:
                self.alg.vis_app_counter     = self.alg.vis_app_counter + 1
                self.alg.vis_app_consecutive = 1
                self.alg.vis_app_last        = 1

        else:
            if self.alg.vis_app_last == 0:
                if(self.alg.vis_app_counter > -self.alg.vis_app_counter_min_max):
                    self.alg.vis_app_counter     = self.alg.vis_app_counter - (1 * self.alg.vis_app_consecutive)
                    self.alg.vis_app_consecutive = self.alg.vis_app_consecutive + 1

                    if(self.alg.vis_app_counter < -self.alg.vis_app_counter_min_max):
                        self.alg.vis_app_counter = -self.alg.vis_app_counter_min_max
                else:
                    pass
            else:
                self.alg.vis_app_counter     = self.alg.vis_app_counter - 1
                self.alg.vis_app_consecutive = 1
                self.alg.vis_app_last        = 0

        if(self.alg.vis_app_counter > self.alg.vis_app_threshold):
            self.alg.vis_app_dist = self.alg.vis_app_dist - (self.alg.vis_app_dist_sub_rate * self.alg.vis_app_dist * 4) #Calculate here the distance of the quad to target
            if(self.alg.vis_app_dist < self.alg.vis_target_dist):
                self.alg.vis_app_dist = self.alg.vis_target_dist #Stop at the target distance. So we aren't kicked out of the visual algorithm

    def updateVisLoc(self, img):
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.alg.ARUCO_DICT, parameters=self.alg.ARUCO_PARAMS)

        if ids is not None:
            for id in ids:
                
                #Set the length of the ID detected.
                if(id[0] == 10):
                    aruco_len = 0.1
                    #Get the rotation vec and translation vec of the camera to the aruco I believe. can use this to control the quad.
                    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners[0], aruco_len, self.alg.camera_matrix, self.alg.camera_dist)

                    cam_pts = np.array([[-tvecs[0][0][1]], 
                                         [tvecs[0][0][0]],
                                         [tvecs[0][0][2]],
                                        [1]])

                    cam2aruco = self.cam2Local(cam_pts)

                    #Using camera 2 aruco points update the Visual Error
                    self.updateVisErr(cam2aruco)

                    #Using the calculated Visual Error check if the quadrotor is in the safe radius
                    self.checkVisAppInRadius()

                    #Depending of the satus of the quadrotor and the safe radius update the visual distance algorithm
                    self.updateVisualDist()

                    #Simplify here. Going directly below. Will need to figure out how to calc err from a spot.
                    self.alg.vs_target_x = self.local_pos.x - (self.alg.x_vis_err * self.alg.vis_P)
                    self.alg.vs_target_y = self.local_pos.y - (self.alg.y_vis_err * self.alg.vis_P)
                    self.alg.vs_target_z = self.local_pos.z - (self.alg.z_vis_err * self.alg.vis_P)

                    #print str("X Loc:" + str(self.local_pos.x) + " X Err:" + str(self.alg.x_vis_err))
                    #print str("Z Loc:" + str(self.local_pos.z) + " Z Err:" + str(self.alg.z_vis_err) + " D: " + str(self.alg.vis_app_dist))

    def fadeToVisLoc(self):
        #Take the Two types and fade and calculate the fade value.
        self.alg.fade_target_x = self.alg.rs_target_x * (1 - self.alg.mode_fade_increment) + self.alg.vs_target_x * (self.alg.mode_fade_increment)
        self.alg.fade_target_y = self.alg.rs_target_y * (1 - self.alg.mode_fade_increment) + self.alg.vs_target_y * (self.alg.mode_fade_increment)
        self.alg.fade_target_z = self.alg.rs_target_z * (1 - self.alg.mode_fade_increment) + self.alg.vs_target_z * (self.alg.mode_fade_increment)

    def cam2Local(self, cam_pts):
        #Here in the function build the Rot matrix of the quadcopter, take XYZ points from camera, [y x z 1] format. Rot_Mat * Cam_Pts will give Aruco wrld pts
        roll  = self.roll
        pitch = self.pitch
        yaw   = self.yaw 

        #print str("Pitch: " + str(pitch) + " Roll: " + str(roll) + " Yaw: " + str(yaw))
        rot_mat = np.array([[math.cos(roll)*math.cos(pitch),   (math.cos(roll)*math.sin(pitch)*math.sin(yaw))-(math.sin(roll)*math.cos(yaw)), math.cos(roll)*math.sin(pitch)*math.cos(yaw)+math.sin(roll)*math.sin(yaw), 0],
                            [math.sin(roll)*math.cos(pitch),  (math.sin(roll)*math.sin(pitch)*math.sin(yaw))+(math.cos(pitch)*math.cos(yaw)), math.sin(roll)*math.sin(pitch)*math.cos(yaw)-math.cos(roll)*math.sin(yaw), 0],
                            [              -math.sin(pitch),                                                   math.cos(pitch)*math.sin(yaw),                                             math.cos(pitch)*math.cos(yaw), 0],
                            [                             0,                                                                               0,                                                                         0, 1]])
        
        Local_pts = np.matmul(rot_mat, cam_pts)

        return Local_pts                    

    def imgCallback(self, img):
        np_arr = np.fromstring(img.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        #Convert image to grey
        grey_im = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)

        #Save this image in our object
        self.alg.img = grey_im

        #print image for viewing purposes
        cv2.imshow('cv_img', grey_im)                    
        cv2.waitKey(2)

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
        #print loc_string
        cnt = cnt + 1
        err = [rvecs[0][0][2], tvecs[0][0][2]]
        
        return err

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
                (rvecs, tvecs, _objPoints) = cv2.aruco.estimatePoseSingleMarkers(corners[cnt], aruco_len, self.alg.camera_matrix, self.alg.camera_dist)

                #Find the tag. Store the xyz error and yaw offset
                self.alg.x_vis_err   = -tvecs[0][0][1]
                self.alg.y_vis_err   =  tvecs[0][0][0]
                self.alg.z_vis_err   =  tvecs[0][0][2]
                self.alg.yaw_vis_err =  rvecs[0][0][2]
                self.mship_visible = 1
                self.mship_not_visible_count = 0

                #Printing for debug purposes
                loc_string = str('yaw: ') + str(rvecs[0][0][2]) + str(' z-dist: ') + str(tvecs[0][0][2])
                #print(loc_string)
                cnt = cnt + 1
        else:
            if(self.mship_not_visible_count > self.mship_debounce and self.mship_visible):
                self.mship_visible = 0
                print('Mship no longer visible')

            self.mship_not_visible_count = self.mship_not_visible_count + 1

        cv2.imshow('cv_img', grey_im)
        cv2.waitKey(2)

    def determineAtRendesvous(self):
        #At Rendesvouz if Quadrotor is withing 0.5m error in x,y,z count up.
        x_err = abs(self.mship_x_err)
        y_err = abs(self.mship_y_err)
        z_err = abs(self.mship_z_err - self.alg.rendesvouz_dist)

        #Error allowed will be the sphere of the quad inside an allowable error sphere that is a function of the rendesvouz distance.
        error_Radius = 0.5 * self.alg.rendesvouz_dist #This value is the error sphere.

        # Check if the quad is within error sphere
        err_dist = (math.sqrt((x_err**2)+(y_err**2)+(z_err**2)))


        #print('Err Distance: ' + str(err_dist))
        
        if(err_dist+self.alg.quad_radius < error_Radius):
            inside_sphere = True

        else:
            inside_sphere = False

        #print str( "X err: " + str(x_err) + " Y err: " + str(y_err) + " Z err: " + str(z_err))
        if(inside_sphere):
            if self.alg.algo_last == 1:
                #If isn't saturated add to counter. If saturated Pass
                if (self.alg.algo_counter < self.alg.algo_counter_sat):
                    self.alg.algo_counter = self.alg.algo_counter + (0.001 * self.alg.algo_consecutive**1.3)
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
            if self.alg.algo_last == 0:
                if(self.alg.algo_counter > -self.alg.algo_counter_sat):
                    self.alg.algo_counter     = self.alg.algo_counter - (0.001 * self.alg.algo_consecutive**1.3)
                    self.alg.algo_consecutive = self.alg.algo_consecutive + 1

                    if(self.alg.algo_counter < -self.alg.algo_counter_sat):
                        self.alg.algo_counter = -self.alg.algo_counter_sat
                else:
                    pass
            else:
                self.alg.algo_counter     = self.alg.algo_counter - 1
                self.alg.algo_consecutive = 1
                self.alg.algo_last        = 0

        if(self.alg.algo_counter > self.alg.algorithm_threshold):
            self.alg.at_rendesvous = 1
        else:
            self.alg.at_rendesvous = 0

    def determineVisualAlg(self):
            if(self.mship_visible):
                self.alg.visual_first_encounter = 1
                #Check if aruco with ID 10 was found the last frame
                if self.alg.vis_last == 1:
                    #If counter isn't saturated add to counter using funciton and increment counter. If Saturated, Pass
                    if (self.alg.vis_counter < self.alg.vis_counter_max_min):
                        self.alg.vis_counter = self.alg.vis_counter + (0.001 * self.alg.vis_consecutive**1.3)
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
                    self.alg.vis_counter = self.alg.vis_counter + (0.001 * self.alg.vis_consecutive**1.3)

            #Doing the inverse if not found.
            else:
                if self.alg.vis_last == 0:
                    #If counter isn't saturated add to counter using function and increment counter. If Saturated pass.
                    if(self.alg.vis_counter > -self.alg.vis_counter_max_min):
                        self.alg.vis_counter = self.alg.vis_counter - (0.001 * self.alg.vis_consecutive**1.3)
                        self.alg.vis_consecutive = self.alg.vis_consecutive + 1

                        #If counter becomes less than the saturation max/min value then set to max/min value.
                        if(self.alg.vis_counter < -self.alg.vis_counter_max_min):
                            self.alg.vis_counter = -self.alg.vis_counter_max_min
                    else:
                        pass
                else:
                    self.alg.vis_last        =  0
                    self.alg.vis_consecutive =  1
                    self.alg.vis_counter = self.alg.vis_counter - (0.001 * self.alg.vis_consecutive**1.3)
    
    def checkMarkerSetMarkerRendesvouzPoint(self):
        if(self.mship_visible):
        #with those points add offset to rendesvouz... Not that this needs to be figured out for rotation frame
            yaw = self.alg.yaw_vis_err

            x_offset = self.alg.x_vis_err * math.cos(yaw) - self.alg.y_vis_err * math.sin(yaw)
            y_offset = self.alg.x_vis_err * math.sin(yaw) + self.alg.y_vis_err * math.cos(yaw)

            self.alg.x_vis_offset = x_offset
            self.alg.y_vis_offset = y_offset
            #print('Offset X: ' + str(self.alg.x_vis_offset) + ' Offset Y: ' + str(self.alg.y_vis_offset))

    def determineEnterVisualMode(self):
        #Check the counters to see if at algorithm location and visual target identified
        if (self.alg.vis_counter > self.alg.algorithm_threshold and self.alg.algo_counter > self.alg.algorithm_threshold):
            #Set visual mode true if thresholds met. 
            self.alg.visual_mode = 1
            self.alg.rendesvous_mode = 0
    
    def determineExitVisualMode(self):
        #Algorithm counter is not used to determine exit of visual mode
        if(self.alg.vis_counter < self.alg.algorithm_threshold):
            #Exit visual mode and Reset first encounter of tag and reset the counter to 0
            self.alg.visual_mode            = 0
            self.alg.visual_first_encounter = 0
            self.alg.vis_counter            = 0
            self.alg.mode_fade_increment    = 0.0
            self.alg.rendesvous_mode        = 1
            self.alg.vis_app_dist           = self.alg.rendesvouz_dist
    
    def logData(self, header=None):
        elapsed_time = time.clock()
        #If the user calls for header return the header or else just return the data. 
        printheader = 'Elapsed_Time local_X local_Y local_Z R_target_X R_target_Y R_target_Z X_err_integrator Y_err_integrator visual_mode visual_first_enc Vis_counter Vis_last Vis_Consecutive ' +\
            'Rendesvous_mode Algo_counter Algo_last Algo_consecutive Vis_app_dist Vis_app_cnt Vis_app_last Vis_app_consecutive quad_radius quad_safe x_vis_err y_vis_err z_vis_err vis_int_x' +\
                'vis_int_y vis_int_z vis_der_x vis_der_y\n'
        printstr = '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f} {6:.3f} {7:.3f} {8:.3f} {9} {10} {11} {12} {13} {14} {15} {16} {17} {18:.3f} {19} {20} {21} {22:.3f} {23:.3f} {24:.3f} {25} {26:.3f} {27:.3f} {28:.3f} {29:.3f} {30:.3f} {31:.3f} {32:.3f} {33:.3f} {34:.3f} {35:.3f} {36:.3f} {37:.3f} {38:.3f} {39:.3f} {40:.3f} {41:.3f} {42:.3f} {43:.3f}\n'.format(\
            elapsed_time,\
            self.local_pos.x, \
            self.local_pos.y,\
            self.local_pos.z,\
            self.alg.rs_target_x_clean,\
            self.alg.rs_target_y_clean,\
            self.alg.rs_target_z_clean,\
            self.alg.x_error_int,\
            self.alg.y_error_int,\
            self.alg.visual_mode,\
            self.alg.visual_first_encounter,\
            self.alg.vis_counter,\
            self.alg.vis_last,\
            self.alg.vis_consecutive,\
            self.alg.rendesvous_mode,\
            self.alg.algo_counter,\
            self.alg.algo_last,\
            self.alg.algo_consecutive,\
            self.alg.vis_app_dist,\
            self.alg.vis_app_counter,\
            self.alg.vis_app_last,\
            self.alg.vis_app_consecutive,\
            self.alg.quad_radius,\
            self.alg.safe_radius,\
            self.alg.err_mag,\
            self.alg.quad_safe,\
            float(self.alg.x_vis_err),\
            float(self.alg.y_vis_err),\
            float(self.alg.z_vis_err),\
            float(self.alg.vis_int_x),\
            float(self.alg.vis_int_y),\
            float(self.alg.vis_int_z),\
            float(self.alg.vis_der_x),\
            float(self.alg.vis_der_y),\
            float(self.sp.position.x),\
            float(self.sp.position.y),\
            float(self.sp.position.z),\
            float(self.alg.fade_target_x),\
            float(self.alg.fade_target_y),\
            float(self.alg.fade_target_z),\
            float(self.alg.mode_fade_increment),\
            float(self.alg.vs_target_x),\
            float(self.alg.vs_target_y),\
            float(self.alg.vs_target_z))

        if header is None:
            return printstr
        else:
            return printheader
    
    def saturateValue(self, value, saturate_min, saturate_max):
        if(value > saturate_max):
            value = saturate_max
        elif(value < saturate_min):
            value = saturate_min
        
        return value
        

