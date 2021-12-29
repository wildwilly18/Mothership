#!/usr/bin/env python
# ROS python API
import roslib
import rospy
import cv2
import math
import numpy as np
import time
import tf.transformations

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
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 3)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed%s"%e

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

        self.yaw   = 0
        self.pitch = 0
        self.roll  = 0


        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        self.sp.yaw        = 0
        self.sp.yaw_rate   = 0

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
            #Camera intrinsic paramaters determined from matlab camera calibration...
            fx = 5.522067755300895e+02
            fy = 7.449299628154979e+02
            cx = 3.479833806300082e+02
            cy = 2.925025107082283e+02
            k1 = -0.425968726364035
            k2 =  0.176325076893194
            
            self.camera_matrix = np.array([[fx, 0, cx],[0, fy, cy], [0, 0, 1]]) #Camera matrix for simulated camera. From fpv_cam.sdf
            self.camera_dist   = np.array([k1, k2, 0, 0]) #Distortion Coefficients for the simlated camera. set to 0 in sim. From fpv_cam.sdf
            self.img           = None

            #Localizing Algorithm info
            self.visual_mode            =    0 #When the quad is determine in position for step 2 this is true
            self.visual_first_encounter =    0 #False until first time a marker is detected. 
            self.vis_counter            =    0 #Track count of visual 
            self.vis_last               =    0 #Keep track if last check aruco was found.
            self.vis_consecutive        =    0 #Track consecutive vis found or not found for algorithm, weights single misses vs multiple frames of misses
            self.vis_counter_max_min    = 1000 #Saturation value for the visual counter
            self.vis_P                  =  0.3 #Proportional gain for our Visual Controller, may need independant X and Y
            self.vis_I                  = 0.08 #Integrator gain for our Visual Controller, may need independant X and Y 
            self.vis_err_x_prev         =    0 #Previous x err
            self.vis_err_y_prev         =    0 #Previous y err
            self.vis_int_x              =    0 #Stored X integrator
            self.vis_int_y              =    0 #Stored Y integrator
            self.vis_int_max            =   50 #Maximum integrator value 

            #Rendesvouz algorithm trackers
            self.rendesvous_mode      =      0    #Set the rendesvous mode on startup. 
            self.algo_counter         =      0    #Here tracks count of each frame where position error from A is < specified value for all dirs
            self.algo_last            =      0    #Track if previous frame aircraft was in the range
            self.algo_consecutive     =      0    #Track consecutive frames of within error range
            self.algo_counter_sat     =   1000    #Saturation value for position
            self.algorithm_threshold  =    750    #Value for algorithm thresholds
            self.mothership_vel       =      0    #Needed for calculating target for the quad
            self.mothership_heading   =      0    #Heading of mothership
            self.pitch_2_match_vel    =      0    #Pitch required by quad to match mship vel
            self.rendesvouz_int       =      0    #Integrate the error for Rendesvouz command point. 
            self.rendesvouz_int_gain  = 0.0005  #Gain for the rendesvouz integrator error.
            self.error_vec_last       =      0    #Error from previous for integrator
            self.x_error_int          =      0    #x integrator value
            self.y_error_int          =      0    #y integrator value
            self.rendesvouz_dist      =      2    #distance in M that the rendesvouz point will be
            self.quad_radius          =    0.1    #Uncertainty sphere radius of the quadrotor


            #Visual Approach Mode trackers
            self.vis_app_dist = self.rendesvouz_dist #Initialize our start with where we rendesvouz too
            self.vis_app_counter           =       0    #Counter to track that we are inside the allowed
            self.vis_app_last              =       0    
            self.vis_app_consecutive       =       0
            self.vis_app_threshold         =     750 #Threshold to turn on or off distance subtraction.
            self.vis_app_counter_min_max   =    1000
            self.vis_app_dist_sub_rate     =   .0001 #Shows how many meters will be removed per frame. runs @~100Hz. this is ~1mm per second. about 1 minute to go 1m
            self.quad_safe                 =   False    #Stor if the quad is in the safe zone for visual servo
            self.x_vis_err                 =       0
            self.y_vis_err                 =       0 
            self.z_vis_err                 =       0

            #Rendeszous target, 2m distance from the ship. Position behind and below to match quads estimated pitch for speed.
            self.rs_target_x = -self.rendesvouz_dist * math.cos(self.pitch_2_match_vel)
            self.rs_target_y =  0
            self.rs_target_z =  self.rendesvouz_dist * math.sin(self.pitch_2_match_vel)

            self.rs_target_x_clean = self.rs_target_x
            self.rs_target_y_clean = self.rs_target_y
            self.rs_target_z_clean = self.rs_target_z
            
            #Visual Servo X, Y & Z positions, start at 2 m dist off the mship. Is a func of mship speed and pithc
            #Below assumption is moving in x dir only
            self.vs_target_x = -self.rendesvouz_dist * math.cos(self.pitch_2_match_vel)
            self.vs_target_y =  0 #eventually will 
            self.vs_target_z =  self.rendesvouz_dist * math.sin(self.pitch_2_match_vel)
    #Class to get the information from the Mothership in the sim.

    class Mothership:
        def __init__(self):
            #Real implementation will need to be a bit different. Need to get the location Lat, Lon, Alt via mavlink msg I believe from QGroundControl

            self.x = 0
            self.y = 0
            self.z = 0
        
            print str( str('Mothership found at, X: ') + str(self.x)+ str(' Y: ') + str(self.y)+ str(' Z: ') + str(self.z))

        def get_sim_location(self):
            #Just get the values from the ros connection to get the model state to use

            #Update the locations to the Mothership object
            self.x = 0
            self.y = 0
            self.z = 8

        def get_sim_world_location(self):
            #This function will need to be updated to transfer from sim location to World Location
            #Just get the values from the ros connection to get the model state to use

            #Update the locations to the Mothership object
            self.x = 0
            self.y = 0
            self.z = 8
	# Callbacks
    ## local position callback
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

        #print str('yaw: ' + str(self.yaw * 180 / 3.14) + ' pitch: ' + str(self.pitch * 180 / 3.14)+ ' roll: ' + str(self.roll * 180 / 3.14))
    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    # Drone heading
    def updateHDG(self, msg):
        self.heading = msg.data

    ## Update setpoint message
    def updateSp(self, x_des, y_des, z_des, yaw=0):
        self.sp.position.x = x_des
        self.sp.position.y = y_des
        self.sp.position.z = z_des
        self.sp.yaw        = yaw

    def globalLoc(self, msg):
        self.sp_glob.latitude  = msg.latitude
        self.sp_glob.longitude = msg.longitude
        self.sp_glob.altitude  = msg.altitude

    def updateRendesvousLoc(self):
        self.mShip.get_sim_location()

        self.alg.rs_target_x_clean = self.mShip.x + (2 * math.sin(0)) #Eventually will be heading
        self.alg.rs_target_y_clean = self.mShip.y + (0) #Eventually will be heading 
        self.alg.rs_target_z_clean = self.mShip.z - (2 * math.cos(self.alg.pitch_2_match_vel))

        #Check error and integrate it.
        #x y plane error
        error_vec = math.sqrt(((self.local_pos.x - self.alg.rs_target_x_clean) ** 2) + ((self.local_pos.y - self.alg.rs_target_y_clean) ** 2))
        if (self.local_pos.x > self.alg.rs_target_x_clean):
            error_dir = -1
        else:
            error_dir =  1

        error_vec = error_dir * error_vec

        #Integrate error
        rendesvouz_int = self.alg.rendesvouz_int + (0.5 * (error_vec + self.alg.error_vec_last))
        self.alg.rendesvouz_int = rendesvouz_int

        #Store error for last error
        self.alg.error_vec_last = error_vec

        self.alg.x_error_int = self.alg.rendesvouz_int * math.cos(0) * self.alg.rendesvouz_int_gain
        y_error_int = error_vec * math.sin(0) * self.alg.rendesvouz_int_gain

        #Add integrator to target.
        #print x_error_int
        self.alg.rs_target_x = self.mShip.x +  0.1 * error_vec + self.alg.x_error_int #Eventually will be including heading
        self.alg.rs_target_y = self.mShip.y  #Eventually will be including heading 
        self.alg.rs_target_z = self.mShip.z - (2 * math.cos(self.alg.pitch_2_match_vel))        

    def updateVisErr(self, cam2aruco):
        d = self.alg.vis_app_dist
        
        x_vis = cam2aruco[0]
        y_vis = cam2aruco[1]
        z_vis = cam2aruco[2]

        #Calculate x y z error of the quad relative to the target.
        self.alg.x_vis_err = (d * math.sin(self.alg.pitch_2_match_vel)) + x_vis #Eventually will add heading
        self.alg.y_vis_err = (0) + y_vis#Eventually will be heading 
        self.alg.z_vis_err = (d * math.cos(self.alg.pitch_2_match_vel)) - z_vis

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

        err_mag = math.sqrt((self.alg.x_vis_err ** 2) + (self.alg.y_vis_err ** 2) + (self.alg.z_vis_err ** 2))

        #print str("Error Mag: " + str(err_mag) + " safe_radius: " + str(safe_radius))

        if(err_mag + quad_rad < safe_radius):
            self.alg.quad_safe = True
        else:
            self.alg.quad_safe = False

    def updateVisualDist(self):
        #Function here will store the confidence of the quads position in approach. It will also calculate D to move the quad closer to the target
        if(self.alg.quad_safe):
            if self.alg.vis_app_last is 1:
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
            if self.alg.vis_app_last is 0:
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
            self.alg.vis_app_dist = self.alg.vis_app_dist - self.alg.vis_app_dist_sub_rate #Calculate here the distance of the quad to target

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

                    #Calculate integrator value
                    self.alg.vis_int_x = 0.5 * (self.alg.x_vis_err + self.alg.vis_err_x_prev)
                    self.alg.vis_int_y = 0.5 * (self.alg.y_vis_err + self.alg.vis_err_y_prev)

                    #Check max integrator values
                    if(self.alg.vis_int_x > self.alg.vis_int_max):
                        self.alg.vis_int_x = self.alg.vis_int_max
                    elif(self.alg.vis_int_x < -self.alg.vis_int_max):
                        self.alg.vis_int_x = -self.alg.vis_int_max

                    if(self.alg.vis_int_y > self.alg.vis_int_max):
                        self.alg.vis_int_y = self.alg.vis_int_max
                    elif(self.alg.vis_int_y < -self.alg.vis_int_max):
                        self.alg.vis_int_y = -self.alg.vis_int_max

                    #After calculating integrator 
                    self.alg.vis_err_x_prev = self.alg.x_vis_err
                    self.alg.vis_err_y_prev = self.alg.y_vis_err

                    #Simplify here. Going directly below. Will need to figure out how to calc err from a spot.
                    self.alg.vs_target_x = self.local_pos.x - (self.alg.x_vis_err * self.alg.vis_P) - (self.alg.vis_int_x * self.alg.vis_I)
                    self.alg.vs_target_y = self.local_pos.y - (self.alg.y_vis_err * self.alg.vis_P) - (self.alg.vis_int_y * self.alg.vis_I)
                    self.alg.vs_target_z = self.local_pos.z - self.alg.z_vis_err

                    #print str("X Loc:" + str(self.local_pos.x) + " X Err:" + str(self.alg.x_vis_err))
                    print str("Z Loc:" + str(self.local_pos.z) + " Z Err:" + str(self.alg.z_vis_err) + " D: " + str(self.alg.vis_app_dist))

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

    def imgCap(self, img):
        #Convert image to grey
        grey_im = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        #Save this image in our object
        self.alg.img = grey_im

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
                #print loc_string
                cnt = cnt + 1
                

        cv2.imshow('cv_img', grey_im)
        cv2.waitKey(2)

    def determineAtRendesvous(self):
        #At Rendesvouz if Quadrotor is withing 0.5m error in x,y,z count up.
        x_err = abs(self.local_pos.x - self.alg.rs_target_x)
        y_err = abs(self.local_pos.y - self.alg.rs_target_y)
        z_err = abs(self.local_pos.z - self.alg.rs_target_z)

        #Error allowed will be the sphere of the quad inside an allowable error sphere that is a function of the rendesvouz distance.
        erro_Radius = 0.5 * self.alg.rendesvouz_dist #This value is the error sphere.

        # Check if the quad is within error sphere
        err_dist = (math.sqrt((x_err**2)+(y_err**2)+(z_err**2)))

        if(err_dist+self.alg.quad_radius < erro_Radius):
            inside_sphere = True

        else:
            inside_sphere = False

        #print str( "X err: " + str(x_err) + " Y err: " + str(y_err) + " Z err: " + str(z_err))
        if(inside_sphere):
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
                if self.alg.vis_last is 0:
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

                    #Doing the inverse if not found.
        #If we have encountered the tag for the first time and we no longer see the tag remove confidence in visual
        elif(self.alg.visual_first_encounter == 1):
            if self.alg.vis_last is 0:
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
            self.alg.vis_app_dist           = self.alg.rendesvouz_dist
    
    def logData(self, header=None):
        elapsed_time = time.clock()
        #If the user calls for header return the header or else just return the data. 
        printheader = 'Elapsed_Time local_X local_Y local_Z R_target_X R_target_Y R_target_Z X_err_integrator Y_err_integrator visual_mode visual_first_enc Vis_counter Vis_last Vis_Consecutive ' +\
            'Rendesvous_mode Algo_counter Algo_last Algo_consecutive \n'
        printstr = '{0:.3f} {1:.3f} {2:.3f} {3:.3f} {4:.3f} {5:.3f} {6:.3f} {7:.3f} {8:.3f} {9} {10} {11:.3f} {12} {13:.3f} {14} {15:.3f} {16} {17:.3f} \n'.format(\
            elapsed_time, self.local_pos.x, self.local_pos.y, self.local_pos.z, self.alg.rs_target_x_clean, self.alg.rs_target_y_clean, self.alg.rs_target_z_clean,         \
                self.alg.x_error_int, self.alg.y_error_int, self.alg.visual_mode, self.alg.visual_first_encounter,self.alg.vis_counter,self.alg.vis_last, \
                self.alg.vis_consecutive, self.alg.rendesvous_mode, self.alg.algo_counter, self.alg.algo_last, self.alg.algo_consecutive)
        if header is None:
            return printstr
        else:
            return printheader