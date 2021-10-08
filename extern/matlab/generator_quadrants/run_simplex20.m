% clear
clc
format longg

addpath(genpath("/home/seedship/TUM/SS21/Thesis/matlab/tbxmanager"));
coef_20 = regression20();

%% Simplex Calculations
lin_min40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_-40/lin.json');
% abs(deg2rad(90) - lin_min40.phi)
[Plist(:, :, 1), Kslist(:, :, 1), Adlist(:,:,1), Bdlist(:,:,1), ss_min40, trim_min40, u_min40, phi_min40] = simplex_20(lin_min40, coef_20, abs(14 - lin_min40.u), deg2rad(30), 50);

lin_min20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_-20/lin.json');
% abs(deg2rad(90) - lin_min20.phi)
[Plist(:, :, 2), Kslist(:, :, 2), Adlist(:,:,2), Bdlist(:,:,2), ss_min20, trim_min20, u_min20, phi_min20] = simplex_20(lin_min20, coef_20, abs(14 - lin_min20.u), deg2rad(30), 50);

lin0 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_0/lin.json');
% deg2rad(90) - lin0.phi
[Plist(:, :, 3), Kslist(:, :, 3), Adlist(:,:,3), Bdlist(:,:,3), ss_0, trim_0, u_0, phi_0] = simplex_20(lin0, coef_20, 6, deg2rad(30), 50);

lin_20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_20/lin.json');
% abs(deg2rad(90) - lin_20.phi)
[Plist(:, :, 4), Kslist(:, :, 4), Adlist(:,:,4), Bdlist(:,:,4), ss_20, trim_20, u_20, phi_20] = simplex_20(lin_20, coef_20, abs(14 - lin_20.u), deg2rad(30), 50);

lin_40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_40/lin.json');
% abs(deg2rad(90) - lin_40.phi)
[Plist(:, :, 5), Kslist(:, :, 5), Adlist(:,:,5), Bdlist(:,:,5), ss_40, trim_40, u_40, phi_40]  = simplex_20(lin_40, coef_20, abs(14 - lin_40.u), deg2rad(30), 50);

%% Reachability Calculations
reachabilityIterations = 1;

lin0_min20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/pitch/-20/lin.json');
% [Adplist(:,:,3), Bdplist(:,:,3), Kr0_min20, Rp0_min20, AL_min20, LL_min20, L_min20, I_min20] = pitch_reachability9(lin0_min20, lin0, coef_20_min20, p_0, 14 - lin0_min20.u, 30 - lin0_min20.u, ...
%     -deg2rad(30), deg2rad(30), -deg2rad(30), deg2rad(30), -50, 50, 1000);
coef_20_min20 = regression20(lin0_min20);
rng(0)
[Krlonglist(:,:,1), Rp0_min20_long, AL_min20_long, LL_min20_long, L_min20_long, IE_min20_long, tu_min20_long] = pitch_reachability_long(lin0_min20, lin0, coef_20_min20, Plist(:, :, 3), 14 - lin0_min20.u, 30 - lin0_min20.u, ...
    -deg2rad(30), deg2rad(30), -50, 50, reachabilityIterations);
rng(0)
[Krlatlist(:,:,1), Rp0_min20_lat, AL_min20_lat, LL_min20_lat, L_min20_lat, IE_min20_lat, tu_min20_lat] = pitch_reachability_lat(lin0_min20, lin0, coef_20_min20, Plist(:, :, 3), -deg2rad(30), deg2rad(30), reachabilityIterations);

lin0_20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/pitch/20/lin.json');
% [Adplist(:,:,1), Bdplist(:,:,1), Kr0_20, Rp0_20, AL_20, LL_20, L_20] = pitch_reachability9(lin0_20, lin0, coef_20_20, p_0, 14 - lin0_20.u, 30 - lin0_20.u, ...
%     -deg2rad(30), deg2rad(30), -deg2rad(30), deg2rad(30), -50, 50, 1000);
coef_20_20 = regression20(lin0_20);
rng(0)
[Krlonglist(:,:,3), Rp0_20_long, AL_20_long, LL_20_long, L_20_long, IE_20_long, tu_20_long] = pitch_reachability_long(lin0_20, lin0, coef_20_20, Plist(:, :, 3), 14 - lin0_20.u, 30 - lin0_20.u, ...
    -deg2rad(30), deg2rad(30), -50, 50, reachabilityIterations);
rng(0)
[Krlatlist(:,:,3), Rp0_20_lat, AL_20_lat, LL_20_lat, L_20_lat, IE_20_lat, tu_20_lat] = pitch_reachability_lat(lin0_20, lin0, coef_20_20, Plist(:, :, 3), -deg2rad(30), deg2rad(30), reachabilityIterations);
