%Roll Pitch Yaw
roll = 0;
pitch = 5;
yaw = 0;

ROT_Mat = [cosd(roll)*cosd(pitch),      cosd(roll)*sind(pitch)*sind(yaw)-sind(roll)*cosd(yaw),  cosd(roll)*sind(pitch)*cosd(yaw)+sind(roll)*sind(yaw), 0;
                       sind(roll)*cosd(pitch),  sind(roll)*sind(pitch)*sind(yaw)+cosd(pitch)*cosd(yaw),  sind(roll)*sind(pitch)*cosd(yaw)-cosd(roll)*sind(yaw),  0;
                                       -sind(pitch),                                                           cosd(pitch)*sind(yaw),                                                     cosd(pitch)*cosd(yaw),  0;
                                                        0,                                                                                              0,                                                                                         0,   1];
cam_x = 2;
cam_y = 0;
Aruco_Cam_Points = [cam_y; cam_x; 5; 1];

%This will return where the Mothership is in relation to the quadcopter. 
Aruco_World_Points = ROT_Mat * Aruco_Cam_Points