function [P, Ks, Ad, Bd, steadystate, trim, u_1, phi_1] = simplex_20(lin, reg_coefs, Du, Dphi, Dh)
u_1 = lin.u;
phi_1 = lin.phi;

F = 100; % AlVolo/uavEE update interval

[Along, Along_corr, Alat_corr, Alat, Blong, Blong_corr, Blat_corr, Blat, Hlong, Hlat, ~, ~, ~] ...
    = generate_submatricies(reg_coefs, lin);
As = [Along, Along_corr, zeros(4,1); Alat_corr, Alat, zeros(4,1); Hlong, Hlat, 0];
Bs = [Blong, Blong_corr; Blat_corr, Blat; zeros(1, 4)];

steadystate = [lin.u; lin.w; lin.q; lin.theta; lin.v; lin.p ; lin.r; lin.phi; 150];
trim = [lin.pitch_ctrl; lin.throttle_ctrl; lin.roll_ctrl; lin.yaw_ctrl];

%% Solver Include
% addpath(genpath("../solvers/YALMIP-master"))
addpath(genpath("../solvers/sdpt3-master"))
% addpath(genpath("../solvers/sedumi-master"))

Q = sdpvar(9, 9);
Z = sdpvar(4, 9);

%% Simplex Limits

uConstraint1 = [-1/Du; zeros(8,1)];
uConstraint2 = [1/Du; zeros(8, 1)];
% thetaConstraint1 = [zeros(3, 1); -1/deg2rad(30); zeros(5,1)];
% thetaConstraint2 = [zeros(3, 1); 1/deg2rad(30); zeros(5,1)];
phiConstraint1 = [zeros(7, 1); -1/Dphi; 0];
phiConstraint2 = [zeros(7, 1); 1/Dphi; 0];
hConstraint1 = [zeros(8, 1); -1/Dh];
hConstraint2 = [zeros(8, 1); 1/Dh];

Dpitch_ctrl = bound(lin.pitch_ctrl);
Dthrottle_ctrl = bound(lin.throttle_ctrl);
Droll_ctrl = bound(lin.roll_ctrl);
Dyaw_ctrl = bound(lin.yaw_ctrl);

elevatorConstraint1 = [-1/Dpitch_ctrl; zeros(3, 1)];
elevatorConstraint2 = [1/Dpitch_ctrl; zeros(3, 1)];
throtleConstraint1 = [0;-1/Dthrottle_ctrl; zeros(2, 1)];
throtleConstraint2 = [0;1/Dthrottle_ctrl; zeros(2, 1)];
aileronConstraint1 = [zeros(2, 1);-1/Droll_ctrl; 0];
aileronConstraint2 = [zeros(2, 1);1/Droll_ctrl; 0];
yawConstraint1 = [zeros(3, 1);-1/Dyaw_ctrl];
yawConstraint2 = [zeros(3, 1);1/Dyaw_ctrl];

%% Limit matrix formulation
uConstraint1 = uConstraint1'*Q*uConstraint1<=1;
uConstraint2 = uConstraint2'*Q*uConstraint2<=1;
% thetaConstraint1 = thetaConstraint1'*Q*thetaConstraint1<=1;
% thetaConstraint2 = thetaConstraint2'*Q*thetaConstraint2<=1;
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
%     thetaConstraint1, thetaConstraint2
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