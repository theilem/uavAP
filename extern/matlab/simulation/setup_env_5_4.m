%% Initial setup and constants

clear
F = 100;
addpath(genpath("/home/seedship/TUM/SS21/Thesis/matlab/tbxmanager"));
addpath(genpath('/home/seedship/TUM/SS21/Thesis/matlab/generator_quadrants'))
load 5_4_gen_qu0.1.mat

%% Linearizations

lin_min40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_-40/lin.json');
lin_min20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_-20/lin.json');
lin_0 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_0/lin.json');
lin_20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_20/lin.json');
lin_40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_40/lin.json');

lin25_min40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_-40/lin.json');
lin25_min20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_-20/lin.json');
lin25_0 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_0/lin.json');
lin25_20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_20/lin.json');
lin25_40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_40/lin.json');

linp_min20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/pitch/-20/lin.json');
linp_20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/pitch/20/lin.json');

%% Regression terms and constants

coef_min40 = regression20(lin_min40);
coef_min20 = regression20(lin_min20);
coef_0 = regression20(lin_0);
coef_20 = regression20(lin_20);
coef_40 = regression20(lin_40);

coef25_min40 = regression25(lin25_min40);
coef25_min20 = regression25(lin25_min20);
coef25_0 = regression25(lin25_0);
coef25_20 = regression25(lin25_20);
coef25_40 = regression25(lin25_40);

coefp_min20 = regression20(linp_min20);
coefp_20 = regression20(linp_20);

%% Indexing arrays
% global idxPhi;
idxPhi = [lin_min40.phi, lin_min20.phi, lin_0.phi, lin_20.phi, lin_40.phi linp_min20.phi linp_20.phi];

% global idxTheta;
idxTheta = [lin_min40.theta, lin_min20.theta, lin_0.theta, lin_20.theta, lin_40.theta linp_min20.theta linp_20.theta];

%% Continuous System Matricies
[Alist(:,:,1), Blist(:,:,1), Clist(:,1)] = generate_full_matrix(coef_min40, lin_min40);
[Alist(:,:,2), Blist(:,:,2), Clist(:,2)] = generate_full_matrix(coef_min20, lin_min20);
[Alist(:,:,3), Blist(:,:,3), Clist(:,3)] = generate_full_matrix(coef_0, lin_0);
[Alist(:,:,4), Blist(:,:,4), Clist(:,4)] = generate_full_matrix(coef_20, lin_20);
[Alist(:,:,5), Blist(:,:,5), Clist(:,5)] = generate_full_matrix(coef_40, lin_40);

[Alist25(:,:,1), Blist25(:,:,1), Clist25(:,1)] = generate_full_matrix(coef25_min40, lin25_min40);
[Alist25(:,:,2), Blist25(:,:,2), Clist25(:,2)] = generate_full_matrix(coef25_min20, lin25_min20);
[Alist25(:,:,3), Blist25(:,:,3), Clist25(:,3)] = generate_full_matrix(coef25_0, lin25_0);
[Alist25(:,:,4), Blist25(:,:,4), Clist25(:,4)] = generate_full_matrix(coef25_20, lin25_20);
[Alist25(:,:,5), Blist25(:,:,5), Clist25(:,5)] = generate_full_matrix(coef25_40, lin25_40);


[Aplist(:,:,1), Bplist(:,:,1), Cplist(:,1)] = generate_full_matrix(coefp_min20, linp_min20);
Aplist(:,:,2) = Alist(:,:,3); Bplist(:,:,2) = Blist(:,:,3); Cplist(:,2) = Clist(:,3);
[Aplist(:,:,3), Bplist(:,:,3), Cplist(:,3)] = generate_full_matrix(coefp_20, linp_20);

%% Discrete System Matricies
[Adplist(:,:,1), Bdplist(:,:,1)] = generateDiscreteSystem(coefp_min20, linp_min20);
Adplist(:,:,2) = Adlist(:,:,3); Bdplist(:,:,2) = Bdlist(:,:,3);
[Adplist(:,:,3), Bdplist(:,:,3)] = generateDiscreteSystem(coefp_20, linp_20);

%% Controller matricies

Qc_20 = zeros(12);
Rc_20 = eye(4);
Qc_20(1,1) = 0.01; % Speed tracking
Qc_20(2,2) = 50; % Pitch tracking
Qc_20(3,3) = 0.1; % Side slip tracking
Qc_20(4,4) = 50; % Roll tracking

[Kclist(:,:,1), Kplist(:,:,1)] = generate_20(lin_min40, coef_min40, Qc_20, Rc_20);
[Kclist(:,:,2), Kplist(:,:,2)] = generate_20(lin_min20, coef_min20, Qc_20, Rc_20);
[Kclist(:,:,3), Kplist(:,:,3)] = generate_20(lin_0, coef_0, Qc_20, Rc_20);
[Kclist(:,:,4), Kplist(:,:,4)] = generate_20(lin_20, coef_20, Qc_20, Rc_20);
[Kclist(:,:,5), Kplist(:,:,5)] = generate_20(lin_40, coef_40, Qc_20, Rc_20);


[Kcplist(:,:,1), Kpplist(:,:,1)] = generate_20(linp_min20, coefp_min20, Qc_20, Rc_20);
Kcplist(:,:,2) = Kclist(:,:,3); Kpplist(:,:,2) = Kplist(:,:,3);
[Kcplist(:,:,3), Kpplist(:,:,3)] = generate_20(linp_20, coefp_20, Qc_20, Rc_20);

%% Actuation Limits

ullist(:, 1) = calculateUpperLimits(lin_min40);
ullist(:, 2) = calculateUpperLimits(lin_min20);
ullist(:, 3) = calculateUpperLimits(lin_0);
ullist(:, 4) = calculateUpperLimits(lin_20);
ullist(:, 5) = calculateUpperLimits(lin_40);

ulplist(:, 1) = calculateUpperLimits(linp_min20);
ulplist(:, 2) = ullist(:, 3);
ulplist(:, 3) = calculateUpperLimits(linp_20);

lllist(:, 1) = calculateLowerLimits(lin_min40);
lllist(:, 2) = calculateLowerLimits(lin_min20);
lllist(:, 3) = calculateLowerLimits(lin_0);
lllist(:, 4) = calculateLowerLimits(lin_20);
lllist(:, 5) = calculateLowerLimits(lin_40);

llplist(:, 1) = calculateLowerLimits(linp_min20);
ullplist(:, 2) = ullist(:, 3);
llplist(:, 3) = calculateLowerLimits(linp_20);

%% Trim

trimlist(:, 1) = [lin_min40.pitch_ctrl, lin_min40.throttle_ctrl, lin_min40.roll_ctrl, lin_min40.yaw_ctrl];
trimlist(:, 2) = [lin_min20.pitch_ctrl, lin_min20.throttle_ctrl, lin_min20.roll_ctrl, lin_min20.yaw_ctrl];
trimlist(:, 3) = [lin_0.pitch_ctrl, lin_0.throttle_ctrl, lin_0.roll_ctrl, lin_0.yaw_ctrl];
trimlist(:, 4) = [lin_20.pitch_ctrl, lin_20.throttle_ctrl, lin_20.roll_ctrl, lin_20.yaw_ctrl];
trimlist(:, 5) = [lin_40.pitch_ctrl, lin_40.throttle_ctrl, lin_40.roll_ctrl, lin_40.yaw_ctrl];

trimlist25(:, 1) = [lin25_min40.pitch_ctrl, lin25_min40.throttle_ctrl, lin25_min40.roll_ctrl, lin25_min40.yaw_ctrl];
trimlist25(:, 2) = [lin25_min20.pitch_ctrl, lin25_min20.throttle_ctrl, lin25_min20.roll_ctrl, lin25_min20.yaw_ctrl];
trimlist25(:, 3) = [lin25_0.pitch_ctrl, lin25_0.throttle_ctrl, lin25_0.roll_ctrl, lin25_0.yaw_ctrl];
trimlist25(:, 4) = [lin25_20.pitch_ctrl, lin25_20.throttle_ctrl, lin25_20.roll_ctrl, lin25_20.yaw_ctrl];
trimlist25(:, 5) = [lin25_40.pitch_ctrl, lin25_40.throttle_ctrl, lin25_40.roll_ctrl, lin25_40.yaw_ctrl];

trimplist(:, 1) = [linp_min20.pitch_ctrl, linp_min20.throttle_ctrl, linp_min20.roll_ctrl, linp_min20.yaw_ctrl];
trimplist(:, 2) = trimlist(:, 3);
trimplist(:, 3) = [linp_20.pitch_ctrl, linp_20.throttle_ctrl, linp_20.roll_ctrl, linp_20.yaw_ctrl];

%% Steady State

sslist(:, 1) = [lin_min40.u, lin_min40.w, lin_min40.q, lin_min40.theta, lin_min40.v, lin_min40.p, lin_min40.r, lin_min40.phi, 0, 0];
sslist(:, 2) = [lin_min20.u, lin_min20.w, lin_min20.q, lin_min20.theta, lin_min20.v, lin_min20.p, lin_min20.r, lin_min20.phi, 0, 0];
sslist(:, 3) = [lin_0.u, lin_0.w, lin_0.q, lin_0.theta, lin_0.v, lin_0.p, lin_0.r, lin_0.phi, 0, 0];
sslist(:, 4) = [lin_20.u, lin_20.w, lin_20.q, lin_20.theta, lin_20.v, lin_20.p, lin_20.r, lin_20.phi, 0, 0];
sslist(:, 5) = [lin_40.u, lin_40.w, lin_40.q, lin_40.theta, lin_40.v, lin_40.p, lin_40.r, lin_40.phi, 0, 0];

sslist25(:, 1) = [lin25_min40.u, lin25_min40.w, lin25_min40.q, lin25_min40.theta, lin25_min40.v, lin25_min40.p, lin25_min40.r, lin25_min40.phi, 0, 0];
sslist25(:, 2) = [lin25_min20.u, lin25_min20.w, lin25_min20.q, lin25_min20.theta, lin25_min20.v, lin25_min20.p, lin25_min20.r, lin25_min20.phi, 0, 0];
sslist25(:, 3) = [lin25_0.u, lin25_0.w, lin25_0.q, lin25_0.theta, lin25_0.v, lin25_0.p, lin25_0.r, lin25_0.phi, 0, 0];
sslist25(:, 4) = [lin25_20.u, lin25_20.w, lin25_20.q, lin25_20.theta, lin25_20.v, lin25_20.p, lin25_20.r, lin25_20.phi, 0, 0];
sslist25(:, 5) = [lin25_40.u, lin25_40.w, lin25_40.q, lin25_40.theta, lin25_40.v, lin25_40.p, lin25_40.r, lin25_40.phi, 0, 0];

ssplist(:, 1) = [linp_min20.u, linp_min20.w, linp_min20.q, linp_min20.theta, linp_min20.v, linp_min20.p, linp_min20.r, linp_min20.phi, 0, 0];
ssplist(:, 2) = sslist(:, 3);
ssplist(:, 3) = [linp_20.u, linp_20.w, linp_20.q, linp_20.theta, linp_20.v, linp_20.p, linp_20.r, linp_20.phi, 0, 0];



%% Reachability tensors
[long20_A, long20_b] = polyArrayToTensor(Rp0_20_long);
[lat20_A, lat20_b] = polyArrayToTensor(Rp0_20_lat);

[longmin20_A, longmin20_b] = polyArrayToTensor(Rp0_min20_long);
[latmin20_A, latmin20_b] = polyArrayToTensor(Rp0_min20_lat);

%% Initial values

% % global upper_lim;
upper_lim = ullist(:, 3);
% % global lower_lim;
lower_lim = lllist(:, 3);
% 
As = Alist(:,:,3);
Bs = Blist(:,:,3);
Cs = Clist(:,3);

Kc = Kclist(:,:,3);
Kp = Kplist(:,:,3);

%% Poly arrays to tensors
[Along_20, blong_20] = polyArrayToTensor(Rp0_20_long);
[Alat_20, blat_20] = polyArrayToTensor(Rp0_20_lat);
[Along_min20, blong_min20] = polyArrayToTensor(Rp0_min20_long);
[Alat_min20, blat_min20] = polyArrayToTensor(Rp0_min20_lat);

%% Helper Functions

function list = calculateUpperLimits(lin)
    list = [1 - lin.pitch_ctrl, 1 - lin.throttle_ctrl, 1 - lin.roll_ctrl, 1 - lin.yaw_ctrl];
end

function list = calculateLowerLimits(lin)
    list = [-1 - lin.pitch_ctrl, -1 - lin.throttle_ctrl, -1 - lin.roll_ctrl, -1 - lin.yaw_ctrl];
end
