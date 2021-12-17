clear
clc
format longg

coef_20 = regression20();
coef_25 = regression25();


Qc_20 = zeros(12);
Rc_20 = eye(4);
Qc_20(1,1) = 0.01; % Speed tracking
Qc_20(2,2) = 50; % Pitch tracking
Qc_20(3,3) = 0.1; % Side slip tracking
Qc_20(4,4) = 50; % Roll tracking

lin20_min40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_-40/lin.json');
lin20_min20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_-20/lin.json');
lin20_0 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_0/lin.json');
lin20_20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_20/lin.json');
lin20_40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_40/lin.json');

lin25_min40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_-40/lin.json');
lin25_min20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_-20/lin.json');
lin25_0 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_0/lin.json');
lin25_20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_20/lin.json');
lin25_40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_40/lin.json');


% lin0_min20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/pitch/-20/lin.json');
% coef_20_min20 = regression20(lin0_min20);
% 
% 
% lin0_20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/pitch/20/lin.json');
% coef_20_20 = regression20(lin0_20);

fprintf('20 min40\n')
[kc20_min40, kp20_min40] = generate_20(lin20_min40, coef_20, Qc_20, Rc_20);
fprintf('20 min20\n')
[kc20_min20, kp20_min20] = generate_20(lin20_min20, coef_20, Qc_20, Rc_20);
fprintf('20 0\n')
[kc20_0, kp20_0] = generate_20(lin20_0, coef_20, Qc_20, Rc_20);
fprintf('20 20\n')
[kc20_20, kp20_20] = generate_20(lin20_20, coef_20, Qc_20, Rc_20);
fprintf('20 40\n')
[kc20_40, kp20_40] = generate_20(lin20_40, coef_20, Qc_20, Rc_20);

fprintf('25 min40\n')
[kc25_min40, kp25_min40] = generate_20(lin25_min40, coef_25, Qc_20, Rc_20);
fprintf('25 min20\n')
[kc25_min20, kp25_min20] = generate_20(lin25_min20, coef_25, Qc_20, Rc_20);
fprintf('25 0\n')
[kc25_0, kp25_0] = generate_20(lin25_0, coef_25, Qc_20, Rc_20);
fprintf('25 20\n')
[kc25_20, kp25_20] = generate_20(lin25_20, coef_25, Qc_20, Rc_20);
fprintf('25 40\n')
[kc25_40, kp25_40] = generate_20(lin25_40, coef_25, Qc_20, Rc_20);

% fprintf('Controller\n')
% printAsJsonEntry(lin20_min40.u, lin20_min40.phi, kc20_min40)
% printAsJsonEntry(lin20_min20.u, lin20_min20.phi, kc20_min20)
% printAsJsonEntry(lin20_0.u, lin20_0.phi, kc20_0)
% printAsJsonEntry(lin20_20.u, lin20_20.phi, kc20_20)
% printAsJsonEntry(lin20_40.u, lin20_40.phi, kc20_40)
% 
% printAsJsonEntry(lin25_min40.u, lin25_min40.phi, kc25_min40)
% printAsJsonEntry(lin25_min20.u, lin25_min20.phi, kc25_min20)
% printAsJsonEntry(lin25_0.u, lin25_0.phi, kc25_0)
% printAsJsonEntry(lin25_20.u, lin25_20.phi, kc25_20)
% printAsJsonEntry(lin25_40.u, lin25_40.phi, kc25_40, 0)
% 
% fprintf('Planner\n')
% printAsJsonEntry(lin20_min40.u, lin20_min40.phi, kp20_min40)
% printAsJsonEntry(lin20_min20.u, lin20_min20.phi, kp20_min20)
% printAsJsonEntry(lin20_0.u, lin20_0.phi, kp20_0)
% printAsJsonEntry(lin20_20.u, lin20_20.phi, kp20_20)
% printAsJsonEntry(lin20_40.u, lin20_40.phi, kp20_40)
% 
% printAsJsonEntry(lin25_min40.u, lin25_min40.phi, kp25_min40)
% printAsJsonEntry(lin25_min20.u, lin25_min20.phi, kp25_min20)
% printAsJsonEntry(lin25_0.u, lin25_0.phi, kp25_0)
% printAsJsonEntry(lin25_20.u, lin25_20.phi, kp25_20)
% printAsJsonEntry(lin25_40.u, lin25_40.phi, kp25_40, 0)

% Printing with only 1 controller
fprintf('Controller\n')
printAsJsonEntry(lin20_0.u, lin20_0.phi, kc20_0)

fprintf('Planner\n')
printAsJsonEntry(lin20_0.u, lin20_0.phi, kp20_0)

function printAsJsonEntry(u, phi, k, comma)
    if nargin<4 || isempty(comma)
      comma=1;
    end
    format longg
    fprintf('{\n')
    fprintf('\t\"setpoint\": {\n')
    fprintf('\t\t\"u\": %e,\n', u)
    fprintf('\t\t\"phi_rad\": %e\n', phi)
    fprintf('\t},\n')
    numbstr = sprintf('%e, ', k);
    numbstr(end-1:end) = [];
    fprintf('\t\"k\": [%s]\n', numbstr)
    fprintf('}')

    if comma
        fprintf(',')
    end
        fprintf('\n')

end