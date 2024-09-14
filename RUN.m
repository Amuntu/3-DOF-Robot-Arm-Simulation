%Use this function to run everything correctly
addpath('\URDF\ROBOT3DOF\urdf');
addpath('\GUI');
addpath('\Kinematics');

answer = StartUp;
switch answer
    case 'FK/IK'
        Forward_Kinematics;
    case 'Jacobain'
        jacobian
    otherwise 
        disp('You didnt choose anything!');
end
