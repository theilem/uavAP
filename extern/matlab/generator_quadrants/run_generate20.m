clear
clc
format longg

[kc_min40, kp_min40, u_min40, phi_min40] = generate_20(19.92684089701237, -0.0015087447522203743, ...
    1.710275573600033, -0.7006502737528185, 0.0654197188864468, ...
    0.027296675922235034, 0.269399662083998, -0.31939226219677713);

[kc_min20, kp_min20, u_min20, phi_min20] = generate_20(19.953032053005206, 0.00015393138893321113, ...
    1.3693996959192307, -0.3489905938601243, 0.06429889158732563, ...
    0.011669829101933198, 0.062125310665494794, -0.17070697804314636);

[kc_0, kp_0, u_0, phi_0] = generate_20(19.959257032508475, 8.529890319855533e-05, ...
    1.2759505956245265, 9.181458768148854e-06, 0.07229047311860616, ...
    0.00018688074160031902, -5.397169272072582e-06, -0.002582645025382552);

[kc_20, kp_20, u_20, phi_20] = generate_20(19.95381890836157, 0.0003369887198820557, ...
    1.3629886994205425, 0.35059347490313253, 0.06416314858042815, ...
    -0.011303424089387058, 0.06018747129516006, 0.164783963946252);

[kc_40, kp_40, u_40, phi_40] = generate_20(19.928577595095373, -0.0005487147160545951, ...
    1.6878515185737344, 0.6981237599378464, 0.0649294904489148, ...
    -0.02622758166310123, 0.2601166796815654, 0.3104817735434434);

fprintf('Controller\n')
printAsJsonEntry(u_min40, phi_min40, kc_min40)
printAsJsonEntry(u_min20, phi_min20, kc_min20)
printAsJsonEntry(u_0, phi_0, kc_0)
printAsJsonEntry(u_20, phi_20, kc_20)
printAsJsonEntry(u_40, phi_40, kc_40, 0)

fprintf('Planner\n')
printAsJsonEntry(u_min40, phi_min40, kp_min40)
printAsJsonEntry(u_min20, phi_min20, kp_min20)
printAsJsonEntry(u_0, phi_0, kp_0)
printAsJsonEntry(u_20, phi_20, kp_20)
printAsJsonEntry(u_40, phi_40, kp_40, 0)

function printAsJsonEntry(u, phi, k, comma)
    if nargin<4 || isempty(comma)
      comma=1;
    end
    format longg
    fprintf('{\n')
    fprintf('\t\"u\": %e,\n', u)
    fprintf('\t\"phi_rad\": %e,\n', phi)
    numbstr = sprintf('%e, ', k);
    numbstr(end-1:end) = [];
    fprintf('\t\"k\": [%s]\n', numbstr)
    fprintf('}')

    if comma
        fprintf(',')
    end
        fprintf('\n')

end