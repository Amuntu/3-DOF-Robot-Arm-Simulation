%Use this function to run everything correctly
addpath('C:\Users\ASUS\Desktop\WORK\3DOF\URDF\ROBOT3DOF\urdf');
addpath('C:\Users\ASUS\Desktop\WORK\3DOF\GUI');
addpath('C:\Users\ASUS\Desktop\WORK\3DOF\Kinematics');
addpath('C:\Users\ASUS\Desktop\WORK\3DOF\Arduino');

answer = StartUp;
switch answer
    case 'FK/IK'
        Forward_Kinematics;
    case 'Jacobain'
        jacobian
    otherwise 
        disp('You didnt choose anything!');
end