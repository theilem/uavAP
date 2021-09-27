function [Ad, Bd] = generateDiscreteSystem(reg_coefs, lin)

[Along, Along_corr, Alat_corr, Alat, Blong, Blong_corr, Blat_corr, Blat, Hlong, Hlat, ~, ~, ~, ~, ~, ~] ...
        = generate_submatricies(reg_coefs, lin);
    A = [Along, Along_corr, zeros(4, 1);
         Alat_corr, Alat, zeros(4,1);
         Hlong, Hlat, 0];
    B = [Blong, Blong_corr;
         Blat_corr, Blat;
         zeros(1, 4)];
     
    F = 100; % AlVolo/uavEE update interval
    continuous = ss(A, B, eye(9), zeros(9, 4));
    discrete = c2d(continuous, 1/F);
    Ad = discrete.A;
    Bd = discrete.B;