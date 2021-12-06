quat = quaternion([-0.0009152, 0.0018271, 0.0002296, -0.99999]);
%quat = quaternion([0.0018271, 0.0009152, -0.0002296, -0.9999]);
eulerRad = euler(quat, 'XYZ', 'frame')
eulerDeg = eulerRad*180/pi()