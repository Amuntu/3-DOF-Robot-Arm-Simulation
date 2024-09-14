function varargout = jacobian(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @jacobian_OpeningFcn, ...
                   'gui_OutputFcn',  @jacobian_OutputFcn, ...
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
function jacobian_OpeningFcn(hObject, eventdata, handles, varargin) %#ok<*INUSL>
handles.output = hObject;
guidata(hObject, handles);
global robot joint q P0 P1 L1 L2 L3 v a q_Limitations d

robot = importrobot('ROBOT3DOF.urdf');

joint = 1:3;

q_Limitations = deg2rad([210 , 180 ,  90 ;
                         -60 ,  0  , -90]);

d = L2+L3;
v = 0;
a = 0;

L1 = .10350;
L2 = .14865;
L3 = .210;

q = [0 pi/2, -pi/2];

P0(1) = L3;
P0(2) = 0;
P0(3) = L1+L2;
P1 = P0;

robot.Bodies{1, joint(1)}.Joint.PositionLimits = [-60*pi/180,210*pi/180];
robot.Bodies{1, joint(2)}.Joint.PositionLimits = [-pi,0];
robot.Bodies{1, joint(3)}.Joint.PositionLimits = [-pi/2,pi/2];
SET_and_SHOW_URDF_ROBOT(handles);
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);
function varargout = jacobian_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;

function Sart_Callback(hObject, eventdata, handles) %#ok<*DEFNU,*INUSD>
global P0 P1 v q L2 L3 Xd Yd Zd

d = norm(P1-P0);
t = d/v;
Xd = (P1(1)-P0(1))/t;
Yd = (P1(2)-P0(2))/t;
Zd = (P1(3)-P0(3))/t;
dt = t/100;
for i = 0:dt:t
    Qd(1) =(Yd*cos(q(1)) - Xd*sin(q(1)))/((L3*cos(q(2) + q(3)) + L2*cos(q(2)))*(cos(q(1))^2 + sin(q(1))^2));
    Qd(2) =-(Zd*sin(q(2) + q(3))*cos(q(1))^2 + Xd*cos(q(2) + q(3))*cos(q(1)) + Zd*sin(q(2) + q(3))*sin(q(1))^2 + Yd*cos(q(2) + q(3))*sin(q(1)))/((L2*cos(q(2) + q(3))*sin(q(2)) - L2*sin(q(2) + q(3))*cos(q(2)))*(cos(q(1))^2 + sin(q(1))^2));
    Qd(3) =(L2*Zd*cos(q(1))^2*sin(q(2)) + L2*Zd*sin(q(1))^2*sin(q(2)) + L3*Xd*cos(q(2) + q(3))*cos(q(1)) + L3*Yd*cos(q(2) + q(3))*sin(q(1)) + L2*Xd*cos(q(1))*cos(q(2)) + L2*Yd*cos(q(2))*sin(q(1)) + L3*Zd*sin(q(2) + q(3))*cos(q(1))^2 + L3*Zd*sin(q(2) + q(3))*sin(q(1))^2)/(L3*(L2*cos(q(2) + q(3))*sin(q(2)) - L2*sin(q(2) + q(3))*cos(q(2)))*(cos(q(1))^2 + sin(q(1))^2));
    
    q = q + Qd*dt;
    
    SET_LIMITATIONS();
    SET_and_SHOW_URDF_ROBOT(handles);
    SET_SLIDERS(handles);
    SET_TEXTBOXES(handles);
    
    pause(dt);
end
set(handles.x0_slider,'value',P1(1));
set(handles.y0_slider,'value',P1(2));
set(handles.z0_slider,'value',P1(3));
set(handles.x1_slider,'value',P1(1));
set(handles.y1_slider,'value',P1(2));
set(handles.z1_slider,'value',P1(3));
set(handles.x0_editText,'string',num2str(P1(1)));
set(handles.y0_editText,'string',num2str(P1(2)));
set(handles.z0_editText,'string',num2str(P1(3)));
set(handles.x1_editText,'string',num2str(P1(1)));
set(handles.y1_editText,'string',num2str(P1(2)));
set(handles.z1_editText,'string',num2str(P1(3)));
function Stop_Callback(hObject, eventdata, handles)
close jacobian
function velocity_slider_Callback(hObject, eventdata, handles)
global v 
v = get(handles.velocity_slider,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);
function velocity_editText_Callback(hObject, eventdata, handles)
global v
v = get(handles.velocity_editText,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);

function x0_slider_Callback(hObject, eventdata, handles)
global P0
P0(1) = get(handles.x0_slider,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);
IK2StartPoint(handles);
function y0_slider_Callback(hObject, eventdata, handles)
global P0
P0(2) = get(handles.y0_slider,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);
IK2StartPoint(handles);
function z0_slider_Callback(hObject, eventdata, handles)
global P0
P0(3) = get(handles.z0_slider,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);
IK2StartPoint(handles);
function x1_slider_Callback(hObject, eventdata, handles)
global P1
P1(1) = get(handles.x1_slider,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);
function y1_slider_Callback(hObject, eventdata, handles)
global P1
P1(2) = get(handles.y1_slider,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);
function z1_slider_Callback(hObject, eventdata, handles)
global P1
P1(3) = get(handles.z1_slider,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);

function x0_editText_Callback(hObject, eventdata, handles)
global P0
P0(1) = get(handles.x0_editText,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);
function y0_editText_Callback(hObject, eventdata, handles)
global P0
P0(2) = get(handles.y0_editText,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);
function z0_editText_Callback(hObject, eventdata, handles)
global P0
P0(3) = get(handles.z0_editText,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);
function x1_editText_Callback(hObject, eventdata, handles)
global P1
P1(1) = get(handles.x1_editText,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);
function y1_editText_Callback(hObject, eventdata, handles)
global P1
P1(2) = get(handles.y1_editText,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);
function z1_editText_Callback(hObject, eventdata, handles)
global P1
P1(3) = get(handles.z1_editText,'value');
SET_SLIDERS(handles);
SET_TEXTBOXES(handles);

function [] = SET_and_SHOW_URDF_ROBOT(handles)
global robot joint q
robot.Bodies{1, joint(1)}.Joint.HomePosition = q(1);
robot.Bodies{1, joint(2)}.Joint.HomePosition = -q(2);
robot.Bodies{1, joint(3)}.Joint.HomePosition = q(3);
%figure(2)
axes(handles.axes2plot)
show(robot);
function [] = SET_SLIDERS(handles)
global P0 P1 v
set(handles.x0_slider,'value',P0(1));
set(handles.y0_slider,'value',P0(2));
set(handles.z0_slider,'value',P0(3));
set(handles.x1_slider,'value',P1(1));
set(handles.y1_slider,'value',P1(2));
set(handles.z1_slider,'value',P1(3));
set(handles. velocity_slider,'value',v);
function [] = SET_TEXTBOXES(handles)
global P0 P1 v
set(handles.x0_editText,'string',num2str(P0(1)));
set(handles.y0_editText,'string',num2str(P0(2)));
set(handles.z0_editText,'string',num2str(P0(3)));
set(handles.x1_editText,'string',num2str(P1(1)));
set(handles.y1_editText,'string',num2str(P1(2)));
set(handles.z1_editText,'string',num2str(P1(3)));
set(handles.velocity_editText,'string',num2str(v));
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
function [] = IK2StartPoint(handles)
global q d L1 P0
t(1) = get(handles.x0_slider,'value');
t(2) = get(handles.y0_slider,'value');
t(3) = get(handles.z0_slider,'value');

x = linspace(P0(1),t(1),20);
y = linspace(P0(2),t(2),20);
z = linspace(P0(3),t(3),20);
for i =1:20
    r = norm([x(i),y(i),(z(i)-L1)]);
    if r>d   
        break
    end
end
P0(1) = x(i-1);
P0(2) = y(i-1);
P0(3) = z(i-1);
q = InverseKinematics(P0(1),P0(2),P0(3));

SET_LIMITATIONS();
SET_and_SHOW_URDF_ROBOT(handles);












function acceleration_editText_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function acceleration_slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function velocity_editText_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function velocity_slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function z1_editText_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function z1_slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function y1_editText_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function y1_slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function x1_editText_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function x1_slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function z0_editText_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function z0_slider_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function y0_editText_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function y0_slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function x0_editText_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function x0_slider_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
