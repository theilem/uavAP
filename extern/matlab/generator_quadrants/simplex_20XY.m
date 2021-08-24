%function [P, Ks, u_1, phi_1] = simplex_20XY(u_1, v_1, w_1, phi_1, theta_1, p_1, q_1, r_1)
u_1 = 19.959257032508475;
v_1 = 8.529890319855533e-05;
w_1 = 1.2759505956245265;
phi_1 = 9.181458768148854e-06;
theta_1 = 0.07229047311860616;
p_1 = 0.00018688074160031902;
q_1 = -5.397169272072582e-06;
r_1 = -0.002582645025382552;
psi_1 = 0;

%% Universal Simplex terms
addpath(genpath("../solvers/YALMIP-master"))
addpath(genpath("../solvers/sdpt3-master"))
addpath(genpath("../solvers/sedumi-master"))

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

pX_u = cos(psi_1)*cos(theta_1);
pX_v = cos(psi_1)*sin(phi_1)*sin(theta_1) - cos(phi_1)*sin(psi_1);
pX_w = sin(phi_1)*sin(psi_1) + cos(phi_1)*cos(psi_1)*sin(theta_1);

pY_u = cos(theta_1)*sin(psi_1);
pY_v = cos(phi_1)*cos(psi_1) + sin(phi_1)*sin(psi_1)*sin(theta_1);
pY_w = cos(phi_1)*sin(psi_1)*sin(theta_1) - cos(psi_1)*sin(phi_1);

pH_u = sin(theta_1);
pH_v = -cos(theta_1)*sin(phi_1);
pH_w = -cos(phi_1)*cos(theta_1);

pX_phi = v_1*(sin(phi_1)*sin(psi_1) + cos(phi_1)*cos(psi_1)*sin(theta_1)) + w_1*(cos(phi_1)*sin(psi_1) - cos(psi_1)*sin(phi_1)*sin(theta_1));
pX_theta = w_1*cos(phi_1)*cos(psi_1)*cos(theta_1) - u_1*cos(psi_1)*sin(theta_1) + v_1*cos(psi_1)*cos(theta_1)*sin(phi_1);
pX_psi = w_1*(cos(psi_1)*sin(phi_1) - cos(phi_1)*sin(psi_1)*sin(theta_1)) - v_1*(cos(phi_1)*cos(psi_1) + sin(phi_1)*sin(psi_1)*sin(theta_1)) - u_1*cos(theta_1)*sin(psi_1);

pX_0 = w_1*(sin(phi_1)*sin(psi_1) + cos(phi_1)*cos(psi_1)*sin(theta_1)) - v_1*(cos(phi_1)*sin(psi_1) - cos(psi_1)*sin(phi_1)*sin(theta_1)) + u_1*cos(psi_1)*cos(theta_1);

pY_u = cos(theta_1)*sin(psi_1);
pY_v = cos(phi_1)*cos(psi_1) + sin(phi_1)*sin(psi_1)*sin(theta_1);
pY_w = cos(phi_1)*sin(psi_1)*sin(theta_1) - cos(psi_1)*sin(phi_1);

pY_phi = -v_1*(cos(psi_1)*sin(phi_1) - cos(phi_1)*sin(psi_1)*sin(theta_1)) - w_1*(cos(phi_1)*cos(psi_1) + sin(phi_1)*sin(psi_1)*sin(theta_1));
pY_theta = w_1*cos(phi_1)*cos(theta_1)*sin(psi_1) - u_1*sin(psi_1)*sin(theta_1) + v_1*cos(theta_1)*sin(phi_1)*sin(psi_1);
pY_psi = w_1*(sin(phi_1)*sin(psi_1) + cos(phi_1)*cos(psi_1)*sin(theta_1)) - v_1*(cos(phi_1)*sin(psi_1) - cos(psi_1)*sin(phi_1)*sin(theta_1)) + u_1*cos(psi_1)*cos(theta_1);

pY_0 = v_1*(cos(phi_1)*cos(psi_1) + sin(phi_1)*sin(psi_1)*sin(theta_1)) - w_1*(cos(psi_1)*sin(phi_1) - cos(phi_1)*sin(psi_1)*sin(theta_1)) + u_1*cos(theta_1)*sin(psi_1);

pH_u = sin(theta_1);
pH_v = -cos(theta_1)*sin(phi_1);
pH_w = -cos(phi_1)*cos(theta_1);

pH_phi = w_1*cos(theta_1)*sin(phi_1) - v_1*cos(phi_1)*cos(theta_1);
pH_theta = u_1*cos(theta_1) + w_1*cos(phi_1)*sin(theta_1) + v_1*sin(phi_1)*sin(theta_1);

pH_0 = u_1*sin(theta_1) - w_1*cos(phi_1)*cos(theta_1) - v_1*cos(theta_1)*sin(phi_1);

%% Sub matricies

Along = [Xu, Xw, Xq, Xtheta; Zu, Zw, Zq, Ztheta; Mu, Mw, Mq, 0; 0, 0, Theta_q, 0];
Along_corr = [Xv, Xp, Xr, 0; Zv, Zp, Zr, Zphi; Mv, Mp, Mr, 0; 0, 0,Theta_r, Theta_phi];
Alat = [Yu, Yw, Yq, Ytheta; Lu, Lw, Lq, 0; Nu, Nw, Nq, 0; 0, 0, Phi_q, Phi_theta];
Alat_corr = [Yv, Yp, Yr, Yphi; Lv, Lp, Lr, 0; Nv, Np, Nr, 0; 0, Phi_p, Phi_r, Phi_phi];

Along_pos = [0, 0, Psi_q, Psi_theta; pX_u, pX_w, 0, pX_theta; pY_u, pY_w, 0, pY_theta; pH_u, pH_w, 0, pH_theta];
Alat_pos = [0, 0, Psi_r, Psi_phi; pX_v, 0, 0, pX_phi; pY_v, 0, 0, pY_phi; pH_v, 0, 0, pH_phi];
A_pos_pos = [ [0; pX_psi; pY_psi; 0],  zeros(4,3)];

long0 = [X0; Z0; M0; Theta_0];
lat0 = [Y0; L0; N0; Phi_0];
pos0 = [Psi_0; pX_0; pY_0; pH_0];

Blong = [Xce, Xct; Zce, Zct; Mce, Mct; zeros(1,2)];
Blong_corr = [Xca, Xcr; Zca, Zcr; Mca, Mcr; zeros(1,2)];
Blat = [Yce, Yct; Lce, Lct; Nce, Nct; zeros(1,2)];
Blat_corr = [Yca, Ycr; Lca, Lcr; Nca, Ncr; zeros(1,2)];

Hlong = [Hu, Hw, 0, Htheta];
Hlat = [Hv, 0, 0, Hphi];

%% System Matrix
As = [Along, Along_corr, zeros(4,4), long0; Alat, Alat_corr, zeros(4,4), lat0; Along_pos, Alat_pos, A_pos_pos, pos0; zeros(1, 12), 1];
Bs = [Blong, Blong_corr; Blat, Blat_corr; zeros(3, 4)];

