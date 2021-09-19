function [Kc, Kp] = generate_20(lin, reg_coefs, Qc, Rc)

%% Constants
g=9.80665;
F = 100; % AlVolo/uavEE update interval

[Along, Along_corr, Alat_corr, Alat, Blong, Blong_corr, Blat_corr, Blat, Hlong, Hlat, Psi_long, Psi_lat, ~, ~, ~, ~] ...
    = generate_submatricies(reg_coefs, lin);


%% Controller Matrix
Ac = [zeros(4,4), [1,0,0,0;0,0,0,1;zeros(2,4)], [zeros(2,4);1,0,0,0;0,0,0,1]; ...
    zeros(4,4), Along, Along_corr;
    zeros(4,4), Alat_corr, Alat];

Bc = [zeros(4, 4);
    Blong, Blong_corr;
    Blat_corr, Blat];

%% Controller Generation
Kc = lqrd(Ac, Bc, Qc, Rc, 1/F);

%% Planner Matrix
Ap = [zeros(2,14), eye(2); zeros(12, 2), Ac - Bc * Kc, zeros(12, 2); zeros(2, 6), [Hlong, Hlat; Psi_long, Psi_lat], zeros(2,2)];
Bp = [zeros(3, 2); -1, 0; 0, 0; 0, -1; zeros(10, 2)];

%% Planner Generation
Qp = zeros(16);
Qp(1,1) = 10; % Altitude tracking
Qp(2,2) = 10; % Heading tracking
Rp = eye(2);
Kp = lqrd(Ap, Bp, Qp, Rp, 1/F);

return

end