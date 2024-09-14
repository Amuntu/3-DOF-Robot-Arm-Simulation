function q = InverseKinematics(px,py,pz)
L1 = .10350; % Links are in [mm]
L2 = .14865;
L3 = .210;

%% theta1
if py==0 && px==0
    q(1) = 0;
else
    q(1) = atan2(py,px);
end
%% theta3
r = sqrt(px^2 + py^2);
D = sqrt((pz - L1)^2 + r^2);
q(3) = -acos((D^2 - L3^2 - L2^2)/(2*L2*L3));
%% theta2
q(2) = -(atan2(r,pz-L1) - atan2(L2+L3*cos(q(3)),L3*sin(q(3))));
end