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

u_dev = 2;
phi_dev = 0.1;

%% Saving to json

% % fprintf('"P":\n')
% rP_min40 = generateRegion(u_min40, phi_min40, p_min40);
% rP_min20 = generateRegion(u_min20, phi_min20, p_min20);
% rP_0 = generateRegion(u_0, phi_0, p_0);
% rP_20 = generateRegion(u_20, phi_20, p_20);
% rP_40 = generateRegion(u_40, phi_40, p_40);
% p_sched = createGainSchedule('P', [rP_min40 rP_min20 rP_0 rP_20 rP_40], u_dev, phi_dev);
% p_json = jsonencode(p_sched, 'PrettyPrint', true);
% 
% % fprintf('"K":\n')
% rK_min40 = generateRegion(u_min40, phi_min40, ks_min40);
% rK_min20 = generateRegion(u_min20, phi_min20, ks_min20);
% rK_0 = generateRegion(u_0, phi_0, ks_0);
% rK_20 = generateRegion(u_20, phi_20, ks_20);
% rK_40 = generateRegion(u_40, phi_40, ks_40);
% k_sched = createGainSchedule('K', [rK_min40 rK_min20 rK_0 rK_20 rK_40], u_dev, phi_dev);
% k_json = jsonencode(k_sched, 'PrettyPrint', true);
% 
% % fprintf('"Ad":\n')
% rA_min40 = generateRegion(u_min40, phi_min40, Ad_min40);
% rA_min20 = generateRegion(u_min20, phi_min20, Ad_min20);
% rA_0 = generateRegion(u_0, phi_0, Ad_0);
% rA_20 = generateRegion(u_20, phi_20, Ad_20);
% rA_40 = generateRegion(u_40, phi_40, Ad_40);
% a_sched = createGainSchedule('Ad', [rA_min40 rA_min20 rA_0 rA_20 rA_40], u_dev, phi_dev);
% a_json = jsonencode(a_sched, 'PrettyPrint', true);
% 
% % fprintf('"Bd":\n')
% rB_min40 = generateRegion(u_min40, phi_min40, Bd_min40);
% rB_min20 = generateRegion(u_min20, phi_min20, Bd_min20);
% rB_0 = generateRegion(u_0, phi_0, Bd_0);
% rB_20 = generateRegion(u_20, phi_20, Bd_20);
% rB_40 = generateRegion(u_40, phi_40, Bd_40);
% b_sched = createGainSchedule('Bd', [rB_min40 rB_min20 rB_0 rB_20 rB_40], u_dev, phi_dev);
% b_json = jsonencode(b_sched, 'PrettyPrint', true);
% 
% % fprintf('"ss":\n')
% rss_min40 = generateRegion(u_min40, phi_min40, ss_min40);
% rss_min20 = generateRegion(u_min20, phi_min20, ss_min20);
% rss_0 = generateRegion(u_0, phi_0, ss_0);
% rss_20 = generateRegion(u_20, phi_20, ss_20);
% rss_40 = generateRegion(u_40, phi_40, ss_40);
% ss_sched = createGainSchedule('ss', [rss_min40 rss_min20 rss_0 rss_20 rss_40], u_dev, phi_dev);
% ss_json = jsonencode(ss_sched, 'PrettyPrint', true);
% 
% % fprintf('"trim":\n')
% rtrim_min40 = generateRegion(u_min40, phi_min40, trim_min40);
% rtrim_min20 = generateRegion(u_min20, phi_min20, trim_min20);
% rtrim_0 = generateRegion(u_0, phi_0, trim_0);
% rtrim_20 = generateRegion(u_20, phi_20, trim_20);
% rtrim_40 = generateRegion(u_40, phi_40, trim_40);
% trim_sched = createGainSchedule('trim', [rtrim_min40 rtrim_min20 rtrim_0 rtrim_20 rtrim_40], u_dev, phi_dev);
% trim_json = jsonencode(trim_sched, 'PrettyPrint', true);

function setpoint = generateSetPoint(u, phi)
    setpoint = struct('u', u, 'phi_rad', phi);
end

function region =  generateRegion(u, phi, k)
    region = struct('setpoint', generateSetPoint(u, phi), 'k', reshape(k, 1, []));
end

function gainSchedule = createGainSchedule(name, regions, u_dev, phi_dev)
    data = struct('regions', regions, 'interpolation_stddev', generateSetPoint(u_dev, phi_dev));
    gainSchedule = struct(name, data);
end

% fprintf('Simplex\n')
% printAsJsonEntry(u_min40, phi_min40, p_min40, ks_min40, Ad_min40, Bd_min40)
% printAsJsonEntry(u_min20, phi_min20, p_min20, ks_min20, Ad_min20, Bd_min20)
% printAsJsonEntry(u_0, phi_0, p_0, ks_0, Ad_0, Bd_0)
% printAsJsonEntry(u_20, phi_20, p_20, ks_20, Ad_20, Bd_20)
% printAsJsonEntry(u_40, phi_40, p_40, ks_40, Ad_40, Bd_40, 0)

% function printAsJsonEntry(u, phi, p, k, ad, bd, comma)
%     if nargin<7 || isempty(comma)
%       comma=1;
%     end
%     fprintf('{\n')
%     fprintf('\t\"u\": %e,\n', u)
%     fprintf('\t\"phi_rad\": %e,\n', phi)
%     numbstr = sprintf('%e, ', p);
%     numbstr(end-1:end) = [];
%     fprintf('\t\"p\": [%s],\n', numbstr)
%     numbstr = sprintf('%e, ', k);
%     numbstr(end-1:end) = [];
%     fprintf('\t\"k\": [%s]\n', numbstr)
%     numbstr = sprintf('%e, ', ad);
%     numbstr(end-1:end) = [];
%     fprintf('\t\"Ad\": [%s]\n', numbstr)
%     numbstr = sprintf('%e, ', bd);
%     numbstr(end-1:end) = [];
%     fprintf('\t\"Bd\": [%s]\n', numbstr)
%     fprintf('}')
% 
%     if comma
%         fprintf(',')
%     end
%         fprintf('\n')
% 
% end