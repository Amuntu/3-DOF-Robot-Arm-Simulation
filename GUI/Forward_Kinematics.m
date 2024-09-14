function varargout = Forward_Kinematics(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Forward_Kinematics_OpeningFcn, ...
                   'gui_OutputFcn',  @Forward_Kinematics_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
function Forward_Kinematics_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;
guidata(hObject, handles);
function varargout = Forward_Kinematics_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;

function Start_FK_pushbutton_Callback(hObject, eventdata, handles)
global robot joint q px py pz L1 L2 L3 q_Limitations d  ARDUINO %#ok<NUSED>
global x y z x_blur y_blur z_blur
load toPlotTheWSasBall x y z x_blur y_blur z_blur
robot = importrobot('ROBOT3DOF.urdf');

joint = 1:3;

L1 = .10350;
L2 = .14865;
L3 = .210;
d = L2+L3;
q_Limitations = deg2rad([210 , 180 ,  90 ;
                         -60 ,  0  , -90]);
       
q = [0 pi/2, -pi/2];

px = L3;
py = 0;
pz = L1+L2;

robot.Bodies{1, joint(1)}.Joint.PositionLimits = [-60*pi/180,210*pi/180];
robot.Bodies{1, joint(2)}.Joint.PositionLimits = [-pi,0];
robot.Bodies{1, joint(3)}.Joint.PositionLimits = [-pi/2,pi/2];

SET_ANGLES_SLIDERS(handles);
SET_POINT_SLIDERS(handles);
SET_TEXTBOXES(handles);
SET_and_SHOW_URDF_ROBOT(handles);
function Stop_FK_pushbutton_Callback(hObject, eventdata, handles)
global ARDUINO %#ok<NUSED>
clearvars
close all
function Defult_PB_Callback(hObject, eventdata, handles)
global q px py pz L1 L2 L3
q = [0 pi/2 -pi/2];
px = L3;
py = 0;
pz = L1+L2;
SET_ANGLES_SLIDERS(handles);
SET_POINT_SLIDERS(handles);
SET_TEXTBOXES(handles);
SET_and_SHOW_URDF_ROBOT(handles);

function theta1_FK_slider_Callback(hObject, eventdata, handles)
global q px py pz 

q(1) = deg2rad(get(handles.theta1_FK_slider,'value'));
q(2) = deg2rad(get(handles.theta2_FK_slider,'value'));
q(3) = deg2rad(get(handles.theta3_FK_slider,'value'));
[px,py,pz] = ForwardKinematics(q(1),q(2),q(3));
    
    SET_and_SHOW_URDF_ROBOT(handles);
    SET_POINT_SLIDERS(handles);
    SET_TEXTBOXES(handles);
%send2arduino(q);
function theta2_FK_slider_Callback(hObject, eventdata, handles)
global q px py pz 

q(1) = deg2rad(get(handles.theta1_FK_slider,'value'));
q(2) = deg2rad(get(handles.theta2_FK_slider,'value'));
q(3) = deg2rad(get(handles.theta3_FK_slider,'value'));
[px,py,pz] = ForwardKinematics(q(1),q(2),q(3));
    
    SET_and_SHOW_URDF_ROBOT(handles);
    SET_POINT_SLIDERS(handles);
    SET_TEXTBOXES(handles);
%send2arduino(q);
function theta3_FK_slider_Callback(hObject, eventdata, handles)
global q px py pz 

q(1) = deg2rad(get(handles.theta1_FK_slider,'value'));
q(2) = deg2rad(get(handles.theta2_FK_slider,'value'));
q(3) = deg2rad(get(handles.theta3_FK_slider,'value'));
[px,py,pz] = ForwardKinematics(q(1),q(2),q(3));
    
    SET_and_SHOW_URDF_ROBOT(handles);
    SET_POINT_SLIDERS(handles);
    SET_TEXTBOXES(handles);
%send2arduino(q);
function Px_IK_slider_Callback(hObject, eventdata, handles)
global q px py pz d L1
t(1) = get(handles.Px_IK_slider,'value');
t(2) = get(handles.Py_IK_slider,'value');
t(3) = get(handles.Pz_IK_slider,'value');

x = linspace(px,t(1),20);
y = linspace(py,t(2),20);
z = linspace(pz,t(3),20);
for i =1:20
    r = norm([x(i),y(i),(z(i)-L1)]);
    if r>d   
        break
    end
end
px = x(i-1);
py = y(i-1);
pz = z(i-1);
q = InverseKinematics(px,py,pz);
SET_LIMITATIONS();
SET_and_SHOW_URDF_ROBOT(handles);
SET_ANGLES_SLIDERS(handles);
set(handles.Py_IK_slider,'value',(py));
set(handles.Pz_IK_slider,'value',(pz));
SET_TEXTBOXES(handles);
%send2arduino(q);
function Py_IK_slider_Callback(hObject, eventdata, handles)
global q px py pz d L1
t(1) = get(handles.Px_IK_slider,'value');
t(2) = get(handles.Py_IK_slider,'value');
t(3) = get(handles.Pz_IK_slider,'value');

x = linspace(px,t(1),20);
y = linspace(py,t(2),20);
z = linspace(pz,t(3),20);
for i =1:20
    r = norm([x(i),y(i),(z(i)-L1)]);
    if r>d   
        break
    end
end
px = x(i-1);
py = y(i-1);
pz = z(i-1);
q = InverseKinematics(px,py,pz);
SET_LIMITATIONS();
SET_and_SHOW_URDF_ROBOT(handles);
SET_ANGLES_SLIDERS(handles);
set(handles.Py_IK_slider,'value',(px));
set(handles.Pz_IK_slider,'value',(pz));
SET_TEXTBOXES(handles);
%send2arduino(q);
function Pz_IK_slider_Callback(hObject, eventdata, handles) %#ok<*INUSL>
global q px py pz d L1
t(1) = get(handles.Px_IK_slider,'value');
t(2) = get(handles.Py_IK_slider,'value');
t(3) = get(handles.Pz_IK_slider,'value');

x = linspace(px,t(1),20);
y = linspace(py,t(2),20);
z = linspace(pz,t(3),20);
for i =1:20
    r = norm([x(i),y(i),(z(i)-L1)]);
    if r>d   
        break
    end
end
px = x(i-1);
py = y(i-1);
pz = z(i-1);
q = InverseKinematics(px,py,pz);
SET_LIMITATIONS();
SET_and_SHOW_URDF_ROBOT(handles);
SET_ANGLES_SLIDERS(handles);
set(handles.Py_IK_slider,'value',(py));
set(handles.Pz_IK_slider,'value',(px));
SET_TEXTBOXES(handles);
%send2arduino(q);
function [] = SET_ANGLES_SLIDERS(handles)
global q
set(handles.theta1_FK_slider,'value',rad2deg(q(1)));
set(handles.theta2_FK_slider,'value',rad2deg(q(2)));
set(handles.theta3_FK_slider,'value',rad2deg(q(3)));
function [] = SET_POINT_SLIDERS(handles)
global px py pz
set(handles.Px_IK_slider,'value',(px));
set(handles.Py_IK_slider,'value',(py));
set(handles.Pz_IK_slider,'value',(pz));
function [] = SET_TEXTBOXES(handles)
global q px py pz
set(handles.theta1_FK_value2show,'string',num2str(rad2deg(q(1))));
set(handles.theta2_FK_value2show,'string',num2str(rad2deg(q(2))));
set(handles.theta3_FK_value2show,'string',num2str(rad2deg(q(3))));
    
set(handles.Px_FK_value2show,'string',num2str(px));
set(handles.Py_FK_value2show,'string',num2str(py));
set(handles.Pz_FK_value2show,'string',num2str(pz));
    
set(handles.Px_IK_value2show,'string',num2str(px));
set(handles.Py_IK_value2show,'string',num2str(py));
set(handles.Pz_IK_value2show,'string',num2str(pz));
    
set(handles.theta1_IK_value2show,'string',num2str(rad2deg(q(1))));
set(handles.theta2_IK_value2show,'string',num2str(rad2deg(q(2))));
set(handles.theta3_IK_value2show,'string',num2str(rad2deg(q(3))));
function [] = SET_LIMITATIONS()
global q q_Limitations 
if q(1)>q_Limitations(1,1)
    q(1) = q_Limitations(1,1);
elseif q(1)<q_Limitations(2,1)
    q(1) = q_Limitations(2,1);
end
if q(2)>q_Limitations(1,2)
    q(2) = q_Limitations(1,2);
elseif q(2)<q_Limitations(2,2)
    q(2) = q_Limitations(2,2);
end
if q(3)>q_Limitations(1,3)
    q(3) = q_Limitations(1,3);
elseif q(3)<q_Limitations(2,3)
    q(3) = q_Limitations(2,3);
end
function [] = SET_and_SHOW_URDF_ROBOT(handles)
global robot joint q
robot.Bodies{1, joint(1)}.Joint.HomePosition = q(1);
robot.Bodies{1, joint(2)}.Joint.HomePosition = -q(2);
robot.Bodies{1, joint(3)}.Joint.HomePosition = q(3);
axes(handles.axes2plot)
show(robot);
hold on
WS_asSphere;
hold off


function Pz_IK_slider_CreateFcn(hObject, eventdata, handles) %#ok<*DEFNU,*INUSD>
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function Py_IK_slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function Px_IK_value2show_Callback(hObject, eventdata, handles)
function Px_IK_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Py_IK_value2show_Callback(hObject, eventdata, handles)
function Py_IK_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Pz_IK_value2show_Callback(hObject, eventdata, handles)
function Pz_IK_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function theta1_IK_value2show_Callback(hObject, eventdata, handles)
function theta1_IK_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function theta2_IK_value2show_Callback(hObject, eventdata, handles)
function theta2_IK_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function theta3_IK_value2show_Callback(hObject, eventdata, handles)
function theta3_IK_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Px_IK_slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function theta1_FK_value2show_Callback(hObject, eventdata, handles)
function theta1_FK_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function theta2_value2show_Callback(hObject, eventdata, handles)
function theta2_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function theta3_value2show_Callback(hObject, eventdata, handles)
function theta3_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function theta2_FK_value2show_Callback(hObject, eventdata, handles)
function theta2_FK_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function theta3_FK_value2show_Callback(hObject, eventdata, handles)
function theta3_FK_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Py_FK_value2show_Callback(hObject, eventdata, handles)
function Py_FK_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function Pz_FK_value2show_Callback(hObject, eventdata, handles)
function Pz_FK_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function theta2_FK_slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function theta3_FK_slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function Px_FK_value2show_Callback(hObject, eventdata, handles)
function Px_FK_value2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function py_text2show_Callback(hObject, eventdata, handles)
function py_text2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function pz_text2show_Callback(hObject, eventdata, handles)
function pz_text2show_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function theta1_FK_slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
