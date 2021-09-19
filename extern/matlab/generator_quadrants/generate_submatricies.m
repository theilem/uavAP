function [Along, Along_corr, Alat_corr, Alat, Blong, Blong_corr, Blat_corr, Blat, Hlong, Hlat, Psi_long, Psi_lat, long0, lat0, H0, Psi_0] ...
    = generate_submatricies(reg_coefs, lin)

%% Constants
g=9.80665;

u_1 = lin.u;
v_1 = lin.v;
w_1 = lin.w;
phi_1 = lin.phi;
theta_1 = lin.theta;
p_1 = lin.p;
q_1 = lin.q;
r_1 = lin.r;

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

Along = [reg_coefs.Xu, reg_coefs.Xw, reg_coefs.Xq, Xtheta; ...
    reg_coefs.Zu, reg_coefs.Zw, reg_coefs.Zq, Ztheta; ...
    reg_coefs.Mu, reg_coefs.Mw, reg_coefs.Mq, 0; ...
    0, 0, Theta_q, 0];
Along_corr = [reg_coefs.Xv, reg_coefs.Xp, reg_coefs.Xr, 0; ...
    reg_coefs.Zv, reg_coefs.Zp, reg_coefs.Zr, Zphi; ...
    reg_coefs.Mv, reg_coefs.Mp, reg_coefs.Mr, 0; ...
    0, 0,Theta_r, Theta_phi];
Alat_corr = [reg_coefs.Yu, reg_coefs.Yw, reg_coefs.Yq, Ytheta; ...
    reg_coefs.Lu, reg_coefs.Lw, reg_coefs.Lq, 0; ...
    reg_coefs.Nu, reg_coefs.Nw, reg_coefs.Nq, 0; ...
    0, 0, Phi_q, Phi_theta];
Alat = [reg_coefs.Yv, reg_coefs.Yp, reg_coefs.Yr, Yphi; ...
    reg_coefs.Lv, reg_coefs.Lp, reg_coefs.Lr, 0; ...
    reg_coefs.Nv, reg_coefs.Np, reg_coefs.Nr, 0; ...
    0, Phi_p, Phi_r, Phi_phi];


long0 = [reg_coefs.X0; reg_coefs.Z0; reg_coefs.M0; Theta_0];
lat0 = [reg_coefs.Y0; reg_coefs.L0; reg_coefs.N0; Phi_0];

Blong = [reg_coefs.Xce, reg_coefs.Xct; reg_coefs.Zce, reg_coefs.Zct; reg_coefs.Mce, reg_coefs.Mct; zeros(1,2)];
Blong_corr = [reg_coefs.Xca, reg_coefs.Xcr; reg_coefs.Zca, reg_coefs.Zcr; reg_coefs.Mca, reg_coefs.Mcr; zeros(1,2)];
Blat_corr = [reg_coefs.Yce, reg_coefs.Yct; reg_coefs.Lce, reg_coefs.Lct; reg_coefs.Nce, reg_coefs.Nct; zeros(1,2)];
Blat = [reg_coefs.Yca, reg_coefs.Ycr; reg_coefs.Lca, reg_coefs.Lcr; reg_coefs.Nca, reg_coefs.Ncr; zeros(1,2)];

Hlong = [Hu, Hw, 0, Htheta];
Hlat = [Hv, 0, 0, Hphi];

Psi_long = [0, 0, Psi_q, Psi_theta];
Psi_lat = [0, 0, Psi_r, Psi_phi];

end