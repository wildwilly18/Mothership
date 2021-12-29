%Roll Pitch Yaw
%quat = quaternion([0, 0, 0, -0.99999]);
quat = quaternion([-0.0013, 0.0034, -0.00067, -0.9999]);
eulerRad = euler(quat, 'XYZ', 'frame')
eulerDeg = eulerRad*180/pi()

roll = eulerRad(1) * 180 / pi();
pitch = eulerRad(2) * 180 / pi();
yaw = eulerRad(3)* 180 / pi();

ROT_Mat = [cosd(roll)*cosd(pitch),      cosd(roll)*sind(pitch)*sind(yaw)-sind(roll)*cosd(yaw),  cosd(roll)*sind(pitch)*cosd(yaw)+sind(roll)*sind(yaw), 0;
                       sind(roll)*cosd(pitch),  sind(roll)*sind(pitch)*sind(yaw)+cosd(pitch)*cosd(yaw),  sind(roll)*sind(pitch)*cosd(yaw)-cosd(roll)*sind(yaw),  0;
                                       -sind(pitch),                                                           cosd(pitch)*sind(yaw),                                                        cosd(pitch)*cosd(yaw),  0;
                                                        0,                                                                                              0,                                                                                         0,   1];
cam_x = -1;
cam_y =  1;
Aruco_Cam_Points = [cam_y; cam_x; 2; 1];

%This will return where the Mothership is in relation to the quadcopter. 
Aruco_World_Points = ROT_Mat * Aruco_Cam_Points