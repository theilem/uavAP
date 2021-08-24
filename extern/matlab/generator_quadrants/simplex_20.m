function [P, Ks, Ad, Bd, u_1, phi_1] = simplex_20(u_1, v_1, w_1, phi_1, theta_1, p_1, q_1, r_1, Du, Dh, Dphi, pitch_ctrl, throttle_ctrl, aileron_ctrl, yaw_ctrl)
% u_1 = 19.959257032508475;
% v_1 = 8.529890319855533e-05;
% w_1 = 1.2759505956245265;
% phi_1 = 9.181458768148854e-06;
% theta_1 = 0.07229047311860616;
% p_1 = 0.00018688074160031902;
% q_1 = -5.397169272072582e-06;
% r_1 = -0.002582645025382552;
% Du = 6;
% Dh = 50;
% Dphi = deg2rad(90) - 9.181458768148854e-06;
% pitch_ctrl = 0.7103086497825895;
% throttle_ctrl = -0.5637563731378527;
% aileron_ctrl = 0.028419362278802075;
% yaw_ctrl = -0.1067113431987168;

%% Constants
g = 9.80665;
F = 100; % AlVolo/uavEE update interval

%% Regressed coefficients
Xu	=	-0.2627559489	;
Xw	=	0.7636313034	;
Xq	=	-0.3503224288	;
Xv	=	0	;
Xp	=	0	;
Xr	=	0	;
Xct	=	3.493111091	;
Xce	=	0	;
Xca	=	0	;
Xcr	=	0	;
X0	=	-0.07302515229	;
			
Zu	=	-0.6128803927	;
Zw	=	-8.144346001	;
Zq	=	17.50574527	;
Zv	=	0	;
Zp	=	0	;
Zr	=	0	;
Zct	=	0	;
Zce	=	5.221055423	;
Zca	=	0	;
Zcr	=	0	;
Z0	=	-0.04941860757	;
			
Mu	=	0.7864886217	;
Mw	=	-13.86457528	;
Mq	=	-24.96965758	;
Mv	=	0	;
Mp	=	0	;
Mr	=	0	;
Mct	=	0.1145797833	;
Mce	=	46.16773474	;
Mca	=	0	;
Mcr	=	0	;
M0	=	-0.02798689117	;
			
Yu	=	0	;
Yw	=	0	;
Yq	=	0	;
Yv	=	-1.246266327	;
Yp	=	1.123655383	;
Yr	=	-20.06950683	;
Yct	=	0	;
Yce	=	0	;
Yca	=	0	;
Ycr	=	-1.55684152	;
Y0	=	-0.08225395555	;
			
Lu	=	0	;
Lw	=	0	;
Lq	=	0	;
Lv	=	0	;
Lp	=	-32.19717634	;
Lr	=	0	;
Lct	=	0	;
Lce	=	0	;
Lca	=	104.5968301	;
Lcr	=	0	;
L0	=	0.4368770043	;
			
Nu	=	0	;
Nw	=	0	;
Nq	=	0.2933180683	;
Nv	=	2.499595746	;
Np	=	-0.5105706621	;
Nr	=	-2.567884351	;
Nct	=	-0.03034984249	;
Nce	=	0	;
Nca	=	-0.5149368263	;
Ncr	=	8.370640962	;
N0	=	-0.02227796081	;

%% Attitude Coefficients
Xtheta = -g * cos(theta_1);
Ztheta = -g * sin(theta_1) * cos(phi_1);
Zphi = -g * cos(theta_1) * sin(phi_1);

Theta_q = cos(phi_1);
Theta_r = -sin(phi_1);
Theta_phi = -q_1 * sin(phi_1) - r_1 * cos(phi_1);
Theta_0 = q_1 * cos(phi_1) - r_1 * sin(phi_1);

Hu = sin(theta_1);
Hw = -cos(theta_1) * cos(phi_1);
Htheta = sin(theta_1) * (w_1 * cos(phi_1) + v_1 * sin(phi_1)) + u_1 * cos(theta_1);
Hv = -cos(theta_1) * sin(phi_1);
Hphi = cos(theta_1) * (w_1 * sin(theta_1) - v_1 * cos(phi_1));
H0 = u_1 * sin(theta_1) - v_1 * cos(theta_1) * sin(phi_1) - w_1 * cos(theta_1) * cos(phi_1);

Ytheta = -g * sin(phi_1) * sin (theta_1);
Yphi = g * cos(phi_1) * cos(theta_1);

Phi_q = sin(phi_1)*tan(theta_1);
Phi_theta = sec(theta_1)^2 * (q_1 * sin(phi_1) + r_1 * cos(phi_1));
Phi_p = 1;
Phi_r = cos(phi_1) * tan(theta_1);
Phi_phi = tan(theta_1) * (q_1 * cos(phi_1) - r_1 * sin(phi_1));
Phi_0 = p_1 + tan(theta_1) * (q_1 * sin(phi_1) + r_1 * cos(phi_1));

Psi_q = sin(phi_1) * sec(theta_1);
Psi_theta = (q_1 * sin(phi_1) + r_1 * cos(phi_1)) * tan(theta_1) * sec(theta_1);
Psi_r = cos(phi_1) * sec(theta_1);
Psi_phi = (q_1 * cos(phi_1) - r_1 * sin(phi_1)) * sec(theta_1);
Psi_0 = (q_1 * sin(phi_1) + r_1 * cos(phi_1)) * sec(theta_1);

%% Sub matricies

Along = [Xu, Xw, Xq, Xtheta; Zu, Zw, Zq, Ztheta; Mu, Mw, Mq, 0; 0, 0, Theta_q, 0];
Along_corr = [Xv, Xp, Xr, 0; Zv, Zp, Zr, Zphi; Mv, Mp, Mr, 0; 0, 0,Theta_r, Theta_phi];
Alat_corr = [Yu, Yw, Yq, Ytheta; Lu, Lw, Lq, 0; Nu, Nw, Nq, 0; 0, 0, Phi_q, Phi_theta];
Alat = [Yv, Yp, Yr, Yphi; Lv, Lp, Lr, 0; Nv, Np, Nr, 0; 0, Phi_p, Phi_r, Phi_phi];

long0 = [X0; Z0; M0; Theta_0];
lat0 = [Y0; L0; N0; Phi_0];

Blong = [Xce, Xct; Zce, Zct; Mce, Mct; zeros(1,2)];
Blong_corr = [Xca, Xcr; Zca, Zcr; Mca, Mcr; zeros(1,2)];
Blat_corr = [Yce, Yct; Lce, Lct; Nce, Nct; zeros(1,2)];
Blat = [Yca, Ycr; Lca, Lcr; Nca, Ncr; zeros(1,2)];

Hlong = [Hu, Hw, 0, Htheta];
Hlat = [Hv, 0, 0, Hphi];

Psi_long = [0, 0, Psi_q, Psi_theta];
Psi_lat = [0, 0, Psi_r, Psi_phi];

%% System Matrix
% with constant
% As = [Along, Along_corr, zeros(4,1), long0; Alat_corr, Alat, zeros(4,1), lat0; Hlong, Hlat, 0, H0; zeros(1, 9), 1];
% Bs = [Blong, Blong_corr; Blat_corr, Blat; zeros(2, 4)];
As = [Along, Along_corr, zeros(4,1); Alat_corr, Alat, zeros(4,1); Hlong, Hlat, 0];
Bs = [Blong, Blong_corr; Blat_corr, Blat; zeros(1, 4)];

%% Solver Include
addpath(genpath("../solvers/YALMIP-master"))
addpath(genpath("../solvers/sdpt3-master"))
addpath(genpath("../solvers/sedumi-master"))

Q = sdpvar(9, 9);
Z = sdpvar(4, 9);

%% Simplex Limits

uConstraint1 = [-1/Du; zeros(8,1)];
uConstraint2 = [1/Du; zeros(8, 1)];
phiConstraint1 = [zeros(7, 1); -1/Dphi; 0];
phiConstraint2 = [zeros(7, 1); 1/Dphi; 0];
hConstraint1 = [zeros(8, 1); -1/Dh];
hConstraint2 = [zeros(8, 1); 1/Dh];

Dpitch_ctrl = bound(pitch_ctrl);
Dthrottle_ctrl = bound(throttle_ctrl);
Daileron_ctrl = bound(aileron_ctrl);
Dyaw_ctrl = bound(yaw_ctrl);

elevatorConstraint1 = [-1/Dpitch_ctrl; zeros(3, 1)];
elevatorConstraint2 = [1/Dpitch_ctrl; zeros(3, 1)];
throtleConstraint1 = [0;-1/Dthrottle_ctrl; zeros(2, 1)];
throtleConstraint2 = [0;1/Dthrottle_ctrl; zeros(2, 1)];
aileronConstraint1 = [zeros(2, 1);-1/Daileron_ctrl; 0];
aileronConstraint2 = [zeros(2, 1);1/Daileron_ctrl; 0];
yawConstraint1 = [zeros(3, 1);-1/Dyaw_ctrl];
yawConstraint2 = [zeros(3, 1);1/Dyaw_ctrl];

%% Limit matrix formulation
uConstraint1 = uConstraint1'*Q*uConstraint1<=1;
uConstraint2 = uConstraint2'*Q*uConstraint2<=1;
phiConstraint1 = phiConstraint1'*Q*phiConstraint1<=1;
phiConstraint2 = phiConstraint2'*Q*phiConstraint2<=1;
hConstraint1 = hConstraint1'*Q*hConstraint1<=1;
hConstraint2 = hConstraint2'*Q*hConstraint2<=1;

elevatorConstraint1 = [1 elevatorConstraint1'*Z;Z'*elevatorConstraint1 Q]>=0;
elevatorConstraint2 = [1 elevatorConstraint2'*Z;Z'*elevatorConstraint2 Q]>=0;
throtleConstraint1 = [1 throtleConstraint1'*Z;Z'*throtleConstraint1 Q]>=0;
throtleConstraint2 = [1 throtleConstraint2'*Z;Z'*throtleConstraint2 Q]>=0;
aileronConstraint1 = [1 aileronConstraint1'*Z;Z'*aileronConstraint1 Q]>=0;
aileronConstraint2 = [1 aileronConstraint2'*Z;Z'*aileronConstraint2 Q]>=0;
yawConstraint1 = [1 yawConstraint1'*Z;Z'*yawConstraint1 Q]>=0;
yawConstraint2 = [1 yawConstraint2'*Z;Z'*yawConstraint2 Q]>=0;

asymphtoticConstraint = [Q*As' + As*Q + Z'*Bs' + Bs*Z <= 0, Q >= 0];

%% Solving

Constraints = [hConstraint1, hConstraint2,
    uConstraint1, uConstraint2,
    phiConstraint1, phiConstraint2
    elevatorConstraint1,elevatorConstraint2,
    throtleConstraint1,throtleConstraint2,
    aileronConstraint1, aileronConstraint2,
    yawConstraint1, yawConstraint2
    asymphtoticConstraint];

% Define Objective
Objective = -logdet(Q);

% % Define solver
Options = sdpsettings('solver','sdpt3');
% Options = sdpsettings('solver','sedumi','sedumi.eps',1e-12);

%  Solve Problem
retval = optimize(Constraints,Objective,Options);
Ks = -value(Z)/value(Q);
P = inv(value(Q));

%% Time delayed system
continuous = ss(As,Bs,eye(9), zeros(9,4));
discrete = c2d(continuous, 1/F);
% sys = idss(As, Bs, eye(11), zeros(11, 4), zeros(11), [zeros(10, 1); 1], 1/100);
Ad = discrete.A;
Bd = discrete.B;

end

function Dctrl = bound (ctrl)
    Dctrl = min(abs(1 - ctrl), abs(1 + ctrl));
end