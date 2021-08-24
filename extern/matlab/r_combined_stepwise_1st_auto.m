clc; clear all;
addpath('~/TUM/SS21/Thesis/SIDPAC')
display("Running: " + mfilename('fullpath'))

%[linearization_path, folder] = uigetfile('*.json');
%linearization_path = string(folder) + linearization_path;
linearization_path = '~/Thesis/multi-SI/linearization/-20/lin.json';

fid = fopen(linearization_path);
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
lin = jsondecode(str);

[datapaths, folder] = uigetfile('*.csv', 'MultiSelect','on');
if ~iscellstr(datapaths)
    datapaths = cellstr(datapaths);
end
mapping = containers.Map();

g = 9.80665;
noiseMag = 0.01;
controlIdx = 0; % Control Type. 0: Throttle, 1: Elevator, 2: Aileron, 3: Rudder

for idx = 1:length(datapaths)
    path = string(string(folder) + string(datapaths(idx)));
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
    
    x = [u, w, data.q, v, data.p, r, ctrl];
    [rows, cols] = size(x);
    x = x + wgn(rows, cols, mag2db(noiseMag));
    
    [y,p,crb,s2,xm,pindx, print_info] = auto_swr(x, data.r_dot, 1, 1, 0.3);
    mapping(string(datapaths(idx))) = print_info;
end

fprintf('Final Results\n')
for key = sort(mapping.keys())
    key = char(key);
    fprintf('%s,', key)
    fprintf('%f,', mapping(key))
    fprintf('\n')
end