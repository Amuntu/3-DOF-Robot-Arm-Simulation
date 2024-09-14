clear pi
syms q1 q2 q3
syms L1 L2 L3
N_DOF = 3;
%% Robot Arm DH Table.
  %    a    alpha     d    theta
DH = [ 0    pi/2      L1    q1 ;
       L2     0       0     q2 ;
       L3     0       0     q3 ];
%% Transformation Matrices for each joint.
T = cell(1,N_DOF);
for i=1:N_DOF
T(1,i) ={ [cos(DH(i,end)),-sin(DH(i,end))*cos(DH(i,2)), sin(DH(i,end))*sin(DH(i,2)),DH(i,1)*cos(DH(i,end));
        sin(DH(i,end)), cos(DH(i,end))*cos(DH(i,2)),-cos(DH(i,end))*sin(DH(i,2)),DH(i,1)*sin(DH(i,end));
             0        ,           sin(DH(i,2))     ,          cos(DH(i,2))      ,      DH(i,3);
             0        ,               0            ,                  0         ,         1            ]};
end   
T01 = cell2sym(T(1));
T12 = cell2sym(T(2));
T23 = cell2sym(T(3));
%% Transformation Matrix for each joint from Base(Zero) frame.
T02 = simplify(T01*T12);
T03 = simplify(T01*T12*T23);
%% (3x1) matrix for each joint position in space.
P01 = T01(1:3,end);
P02 = T02(1:3,end);
P03 = T03(1:3,end);
%% (3x3) matrix for each joint Rotation in space.
R00 = sym(eye(3));
R01 = T01(1:3,1:3);
R02 = T02(1:3,1:3);
R03 = T03(1:3,1:3);
%% End effector position (Forwarm Kenimatics equations).
px = P03(1);
py = P03(2);
pz = P03(3);