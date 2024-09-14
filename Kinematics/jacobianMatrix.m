TransformationMatrices;

syms Vx Vy Vz Wx Wy Wz 
syms q1d q2d q3d

J = [cross(R00*[0;0;1],(P03)) , cross(R01*[0;0;1],(P03-P01)) , cross(R02*[0;0;1],(P03-P02)) ;
            R00*[0;0;1]       ,          R01*[0;0;1]         ,           R02*[0;0;1]        ];
 
Pd = [ Vx ; Vy ; Vz ; Wx ; Wy ; Wz ];

t = eye(6);
J = [J , t(:,4:end)]; %make it square matrix 6x6

inverseJacobian = inv(J) * Pd;

Qd = simplify(inverseJacobian);