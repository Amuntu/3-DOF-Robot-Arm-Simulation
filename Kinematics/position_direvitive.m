syms X(t) Y(t) Z(t) q1(t) q2(t) q3(t) L1 L2 L3

EQ = sym(zeros(3,1));
EQ(1) = X == cos(q1)*(L3*cos(q2 + q3) + L2*cos(q2));
EQ(2) = Y == sin(q1)*(L3*cos(q2 + q3) + L2*cos(q2));
EQ(3) = Z ==  L1 + L3*sin(q2 + q3) + L2*sin(q2);

EQ = diff(EQ);

 