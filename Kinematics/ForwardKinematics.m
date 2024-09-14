function [px,py,pz] = ForwardKinematics(q1,q2,q3)
L1 = .10350; % Links are in [mm]
L2 = .14865;
L3 = .210;
px = cos(q1)*(L3*cos(q2 + q3) + L2*cos(q2));
py = sin(q1)*(L3*cos(q2 + q3) + L2*cos(q2));
pz = L1 + L3*sin(q2 + q3) + L2*sin(q2);
end