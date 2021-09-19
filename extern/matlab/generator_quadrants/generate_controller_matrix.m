function [Ac, Bc] = generate_controller_matrix(reg_coefs, lin)
    [Along, Along_corr, Alat_corr, Alat, Blong, Blong_corr, Blat_corr, Blat, ~, ~, ~, ~, ~, ~, ~, ~] ...
        = generate_submatricies(reg_coefs, lin);


    %% Controller Matrix
    Ac = [zeros(4,4), [1,0,0,0;0,0,0,1;zeros(2,4)], [zeros(2,4);1,0,0,0;0,0,0,1]; ...
        zeros(4,4), Along, Along_corr;
        zeros(4,4), Alat_corr, Alat];

    Bc = [zeros(4, 4);
        Blong, Blong_corr;
        Blat_corr, Blat];

return