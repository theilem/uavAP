function [A, B, C] = generate_full_matrix(reg_coefs, lin)
[Along, Along_corr, Alat_corr, Alat, Blong, Blong_corr, Blat_corr, Blat, Hlong, Hlat, Psi_long, Psi_lat, long0, lat0, H0, Psi_0] ...
    = generate_submatricies(reg_coefs, lin);
    A = [Along, Along_corr, zeros(4, 2);
        Alat_corr, Alat, zeros(4, 2);
        [Hlong, Hlat; Psi_long, Psi_lat], zeros(2, 2)];

    B = [Blong, Blong_corr;
        Blat_corr, Blat;
        zeros(2, 4)];
    
    C = [long0; lat0; H0; Psi_0];

end