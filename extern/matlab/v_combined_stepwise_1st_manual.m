clc; clear all;
addpath('~/TUM/SS21/Thesis/SIDPAC')
display("Running: " + mfilename('fullpath'))

%[linearization_path, folder] = uigetfile('*.json');
%linearization_path = string(folder) + linearization_path;
linearization_path = '~/Thesis/multi-SI/linearization/20/lin.json';

fid = fopen(linearization_path);
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
lin = jsondecode(str);

[datapath, folder] = uigetfile('*.csv');
mapping = containers.Map();

g = 9.80665;
noiseMag = 0.01;
controlIdx = 1; % Control Type. 0: Throttle, 1: Elevator, 2: Aileron, 3: Rudder

path = string(string(folder) + string(datapath));
display("File is: " + path)
data = readtable(path);

u = data.u - lin.u;
w = data.w - lin.w;
theta = data.theta - lin.theta;
v = data.v - lin.v;
r = data.r - lin.r;
phi = data.phi - lin.phi;
if controlIdx == 0
    ctrl = data.throttle_ctrl - lin.throttle_ctrl;
    display("Control is Throttle")
elseif controlIdx == 1
    ctrl = data.pitch_ctrl - lin.pitch_ctrl;
    display("Control is Elevator")
elseif controlIdx == 2
    ctrl = data.roll_ctrl - lin.roll_ctrl;
    display("Control is Aileron")
else
    ctrl = data.yaw_ctrl - lin.yaw_ctrl;
    display("Control is Rudder")
end
v_dot = data.v_dot + (g * sin(lin.phi) * sin(lin.theta)) * theta - (g * cos(lin.phi) * cos(lin.theta)) * phi ;
x = [u, w, data.q, v, data.p, r, ctrl];
[rows, cols] = size(x);
x = x + wgn(rows, cols, mag2db(noiseMag));
[y,p,crb,s2,xm,pindx] = swr(x, v_dot, 1);