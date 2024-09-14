clearvars
syms Xd Yd Zd q1d q2d q3d q1 q2 q3 L1 L2 L3

EQ = sym(zeros(3,1));

EQ(1) = Xd == - cos(q1)*(L2*q2d*sin(q2) + L3*sin(q2 + q3)*(q2d + q3d)) - q1d*sin(q1)*(L3*cos(q2 + q3) + L2*cos(q2));
EQ(2) = Yd == q1d*cos(q1)*(L3*cos(q2 + q3) + L2*cos(q2)) - sin(q1)*(L2*q2d*sin(q2) + L3*sin(q2 + q3)*(q2d + q3d));
EQ(3) = Zd == L3*cos(q2 + q3)*(q2d + q3d) + L2*q2d*cos(q2);
 
[q1d,q2d,q3d] = solve(EQ(1),EQ(2),EQ(3),q1d,q2d,q3d)
 