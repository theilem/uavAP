%% Setup
clear
clc
format longg

%% Constants
g=9.80665;
ELEVATOR_LIMIT = -16; % sign flip for elevator due to X-plane conventions
RUDDER_LIMIT = 22; % no sign flip for rudder due to X-plane conventions
AILERON_LIMIT = -17; % sign flip for aileron due to X-plane conventions
F = 100; % AlVolo/uavEE update interval

%% Trim
u_1 = 22.5794785457791;
w_1 = 1.094520935399559;
theta_1 = 0.04183938913580468;
pitch_ctrl = 0.36749821440099206;
throttle_ctrl = -0.48589201718311287;

v_1 = 0;
phi_1 = 0;
roll_ctrl = 0.006927637640779975;
yaw_ctrl = 0;

p_1 = 0;
q_1 = 0;
r_1 = 0;

%% Longitudinal System coefficients
Xu	=	-0.2749084918	;
Xw	=	1.000327001	;
Xq	=	-0.2383107127	;
Xde	=	0.01692767261	* ELEVATOR_LIMIT;
X0	=	0.09153453309	;
			
Zu	=	-0.4523404264	;
Zw	=	-10.04600056	;
Zq	=	0.02164371184	;
Zde	=	-0.4829286784	* ELEVATOR_LIMIT;
Z0	=	-0.1898993745	;
			
Mu 	=	0.7498375749	;
Mw 	=	-17.17353139	;
Mq 	=	-10.87913215	;
Mde	=	-3.726347663	* ELEVATOR_LIMIT;
M0	=	-0.09673854488	;

%% Lateral System Coefficients
Yv	=	-1.27811652	;
Yp	=	0.123347866	;
Yr	=	-1.379598873	;
Yda	=	-0.06887207651	*AILERON_LIMIT;
Ydr	=	-0.06224916973	*RUDDER_LIMIT;
Y0	=	-0.1436961795	;
			
Lv	=	-1.562993756	;
Lp	=	-29.71722958	;
Lr	=	5.430432741	;
Lda	=	-8.818825339	*AILERON_LIMIT;
Ldr	=	-0.09978910137	*RUDDER_LIMIT;
L0	=	0.3607901975	;
			
Nv	=	3.110896148	;
Np	=	-1.053825321	;
Nr	=	-2.920187745	;
Nda	=	0.04744663703	*AILERON_LIMIT;
Ndr	=	0.4456565752	*RUDDER_LIMIT;
N0	=	-0.5810910582	;

%% Longitudinal Correlated Terms
Xv = 0;
Xp = 0;
Xr = 0;
Xda = 0;
Xdr = 0;

Zv = 0;
Zp = 0;
Zr = 0;
Zda = 0;
Zdr = 0;

Mv = 0;
Mp = 0;
Mr = 0;
Mda = 0;
Mdr = 0;

%% Lateral Correlated Terms
Yu = 0;
Yw = 0;
Yq = 0;
Yde = 0;

Lu = 0;
Lw = 0;
Lq = 0;
Lde = 0;

Nu = 0;
Nw = 0;
Nq = 0;
Nde = 0;

%Throttle Terms
Xdt = 5.28285098996209;
Zdt = 0;
Mdt = 0;
Ydt = 0;
Ldt = 0;
Ndt = 0;

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
Alat = [Yu, Yw, Yq, Ytheta; Lu, Lw, Lq, 0; Nu, Nw, Nq, 0; 0, 0, Phi_q, Phi_theta];
Alat_corr = [Yv, Yp, Yr, Yphi; Lv, Lp, Lr, 0; Nv, Np, Nr, 0; 0, Phi_p, Phi_r, Phi_phi];

long0 = [X0; Z0; M0; Theta_0];
lat0 = [Y0; L0; N0; Psi_0];

Blong = [Xde, Xdt; Zde, Zdt; Mde, Mdt; zeros(1,2)];
Blong_corr = [Xda, Xdr; Zda, Zdr; Mda, Mdr; zeros(1,2)];
Blat = [Yde, Ydt; Lde, Ldt; Nde, Ndt; zeros(1,2)];
Blat_corr = [Yda, Ydr; Lda, Ldr; Nda, Ndr; zeros(1,2)];

Hlong = [Hu, Hw, 0, Htheta];
Hlat = [Hv, 0, 0, Hphi];

Psi_long = [0, 0, Psi_q, Psi_theta];
Psi_lat = [0, 0, Psi_r, Psi_phi];

%% System Matrix
As = [Along, Along_corr, zeros(4,2), long0; Alat, Alat_corr, zeros(4,2), lat0; [Hlong, Hlat; Psi_long, Psi_lat], zeros(2,2), [H0; Psi_0]];
Bs = [Blong, Blong_corr; Blat, Blat_corr; zeros(2, 4)];

%% Controller Matrix
Ac = [zeros(4,4), [1,0,0,0;0,0,0,1;zeros(2,4)], [zeros(2,4);1,0,0,0;0,0,0,1]; ...
    zeros(4,4), Along, Along_corr;
    zeros(4,4), Alat, Alat_corr];

Bc = [zeros(4, 4);
    Blong, Blong_corr;
    Blat, Blat_corr];
%{
Ac = [zeros(4,4), [1,0,0,0;0,0,0,1;zeros(2,4)], [zeros(2,4);1,0,0,0;0,0,0,1],zeros(4,1); ...
    zeros(4,4), [Xu, Xw, Xq, Xtheta; Zu, Zw, Zq, Ztheta; Mu, Mw, Mq, 0; 0, 0, Theta_q, 0], ...
    [Xv, Xp, Xr, 0; Zv, Zp, Zr, Zphi; Mv, Mp, Mr, 0; 0, 0,Theta_r, Theta_phi], [X0; Z0; M0; Theta_0];
    zeros(4,4), [Yu, Yw, Yq, Ytheta; Lu, Lw, Lq, 0; Nu, Nw, Nq, 0; 0, 0, Phi_q, Phi_theta], ...
    [Yv, Yp, Yr, Yphi; Lv, Lp, Lr, 0; Nv, Np, Nr, 0; 0, Phi_p, Phi_r, Phi_phi], [Y0; L0; N0; Phi_0]; ...
    zeros(1,13)];

Bc = [zeros(4, 4);
    Xde, Xdt, Xda, Xdr;
    Zde, Zdt, Zda, Zdr;
    Mde, Mdt, Mda, Mdr;
    zeros(1, 4);
    Yde, Ydt, Yda, Ydr;
    Lde, Ldt, Lda, Ldr;
    Nde, Ndt, Nda, Ndr;
    zeros(2, 4)];
%}


%% Controller Generation

Qc = zeros(12);
Rc = eye(4);
Qc(1,1) = 0.1; % Speed tracking
Qc(2,2) = 50; % Pitch tracking
Qc(3,3) = 0.00001; % Side slip tracking
Qc(4,4) = 50; % Roll tracking
Kc = lqrd(Ac, Bc, Qc, Rc, 1/F);

%% Planner Matrix
Ap = [zeros(2,14), eye(2); zeros(12, 2), Ac - Bc * Kc, zeros(12, 2); zeros(2, 6), [Hlong, Hlat; Psi_long, Psi_lat], zeros(2,2)];
Bp = [zeros(3, 2); -1, 0; 0, 0; 0, -1; zeros(10, 2)];

%% Planner Generation
Qp = zeros(16);
Qp(1,1) = 0.1; % Altitude tracking
Qp(2,2) = 0.1; % Heading tracking
Rp = eye(2);
Kp = lqrd(Ap, Bp, Qp, Rp, 1/F);

%% Printing

fprintf('\"k (controller)\": [');fprintf('%.17f, ', Kc);fprintf('],\n')
fprintf('\"k (planner)\": [');fprintf('%.17f, ', Kp);fprintf('],\n')

