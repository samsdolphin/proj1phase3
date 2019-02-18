function [F, M] = controller(t, s, s_des)

% s=[x, y, z, v_x, v_y, v_z, quaternion, p, q, r];

global params

m = params.mass;
g = params.grav;
I = params.I;

% You should calculate the output F and M
Kd1=3;
Kp1=5;
Kd2=3;
Kp2=5;
Kd3=10;
Kp3=10;
KpPhi=1000;
KpTheta=1000;
Kpyaw=150;
KdPhi=15;
KdTheta=15;
Kdyaw=15;

r1_des2=Kd1*(s_des(4)-s(4))+Kp1*(s_des(1)-s(1)); % second derivative of desired x
r2_des2=Kd2*(s_des(5)-s(5))+Kp2*(s_des(2)-s(2));
r3_des2=Kd3*(s_des(6)-s(6))+Kp3*(s_des(3)-s(3));

Rot_des = QuatToRot([s_des(7), s_des(8), s_des(9), s_des(10)]');
[phi_des, theta_des, yaw_des] = RotToRPY_ZXY(Rot_des); % desired angle calculated from quaternion

Rot = QuatToRot([s(7),s(8),s(9),s(10)]');
[phi,theta,yaw] = RotToRPY_ZXY(Rot);

phi_des = 1/g*(r1_des2*sin(yaw)-r2_des2*cos(yaw));
theta_des = 1/g*(r1_des2*cos(yaw)+ r2_des2*sin(yaw));

phi1 = s(11);
theta1 = s(12);
yaw1 = s(13);

phi_des1 = 0;
theta_des1 = 0;
yaw_des1 = 0;

dYaw = yaw_des-yaw;

% dYaw
if(dYaw >= pi)
    dYaw = -( 2*pi - dYaw);
elseif(dYaw <= -pi)
    dYaw = 2*pi + dYaw;
end

phi_des2 = KpPhi*(phi_des-phi)+KdPhi*(phi_des1-phi1); % second derivative of desired angle phi
theta_des2 = KpTheta*(theta_des-theta)+KdTheta*(theta_des1-theta1);
yaw_des2 = Kpyaw*(dYaw)+Kdyaw*(yaw_des1-yaw1);

F = m*(g+r3_des2);

M = I*[phi_des2, theta_des2, yaw_des2]'+cross([s(11), s(12), s(13)]',I*[s(11), s(12), s(13)]');

end
