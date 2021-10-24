% fprintf('"Ad":\n')
theta_dev = 0.1;
phi_dev = 0.1;

rA_min40 = generateRegion(lin_min40.theta, lin_min40.phi, Adlist(:,:,1));
rA_min20 = generateRegion(lin_min20.theta, lin_min20.phi, Adlist(:,:,2));
rA_0 = generateRegion(lin_0.theta, lin_0.phi, Adlist(:,:,3));
rA_20 = generateRegion(lin_20.theta, lin_20.phi, Adlist(:,:,4));
rA_40 = generateRegion(lin_40.theta, lin_40.phi, Adlist(:,:,5));

rAmin20 = generateRegion(lin0_min20.theta, lin0_min20.phi, Adplist(:,:,1));
rA20 = generateRegion(lin0_20.theta, lin0_20.phi, Adplist(:,:,3));
a_sched = createGainSchedule({rA_min40 rA_min20 rA_0 rA_20 rA_40 rAmin20 rA20}, theta_dev, phi_dev);

% fprintf('"Bd":\n')
rB_min40 = generateRegion(lin_min40.theta, lin_min40.phi, Bdlist(:,:,1));
rB_min20 = generateRegion(lin_min20.theta, lin_min20.phi, Bdlist(:,:,2));
rB_0 = generateRegion(lin_0.theta, lin_0.phi, Bdlist(:,:,3));
rB_20 = generateRegion(lin_20.theta, lin_20.phi, Bdlist(:,:,4));
rB_40 = generateRegion(lin_40.theta, lin_40.phi, Bdlist(:,:,5));

rBmin20 = generateRegion(lin0_min20.theta, lin0_min20.phi, Bdplist(:,:,1));
rB20 = generateRegion(lin0_20.theta, lin0_20.phi, Bdplist(:,:,3));
b_sched = createGainSchedule({rB_min40 rB_min20 rB_0 rB_20 rB_40 rBmin20 rB20}, theta_dev, phi_dev);

%fprintf('"C":\n')
rC_min40 = generateRegion(lin_min40.theta, lin_min40.phi, Clist(1:9,1));
rC_min20 = generateRegion(lin_min20.theta, lin_min20.phi, Clist(1:9,2));
rC_0 = generateRegion(lin_0.theta, lin_0.phi, Clist(1:9,3));
rC_20 = generateRegion(lin_20.theta, lin_20.phi, Clist(1:9,4));
rC_40 = generateRegion(lin_40.theta, lin_40.phi, Clist(1:9,5));

rCmin20 = generateRegion(lin0_min20.theta, lin0_min20.phi, Cplist(1:9,1));
rC20 = generateRegion(lin0_20.theta, lin0_20.phi, Cplist(1:9,3));
c_sched = createGainSchedule({rC_min40 rC_min20 rC_0 rC_20 rC_40 rCmin20 rC20}, theta_dev, phi_dev);

% fprintf('"ss":\n')
ss_min40 = generateRegion(lin_min40.theta, lin_min40.phi, sslist(1:9,1));
ss_min20 = generateRegion(lin_min20.theta, lin_min20.phi, sslist(1:9,2));
ss_0 = generateRegion(lin_0.theta, lin_0.phi, sslist(1:9,3));
ss_20 = generateRegion(lin_20.theta, lin_20.phi, sslist(1:9,4));
ss_40 = generateRegion(lin_40.theta, lin_40.phi, sslist(1:9,5));

ssmin20 = generateRegion(lin0_min20.theta, lin0_min20.phi, ssplist(1:9, 1));
ss20 = generateRegion(lin0_20.theta, lin0_20.phi, ssplist(1:9, 3));
ss_sched = createGainSchedule({ss_min40 ss_min20 ss_0 ss_20 ss_40 ssmin20 ss20}, theta_dev, phi_dev);

% fprintf('"trim":\n')
trim_min40 = generateRegion(lin_min40.theta, lin_min40.phi, trimlist(:,1));
trim_min20 = generateRegion(lin_min20.theta, lin_min20.phi, trimlist(:,2));
trim_0 = generateRegion(lin_0.theta, lin_0.phi, trimlist(:,3));
trim_20 = generateRegion(lin_20.theta, lin_20.phi, trimlist(:,4));
trim_40 = generateRegion(lin_40.theta, lin_40.phi, trimlist(:,5));

trimmin20 = generateRegion(lin0_min20.theta, lin0_min20.phi, trimplist(:,1));
trim20 = generateRegion(lin0_20.theta, lin0_20.phi, trimplist(:,3));
trim_sched = createGainSchedule({trim_min40 trim_min20 trim_0 trim_20 trim_40 trimmin20 trim20}, theta_dev, phi_dev);

% fprintf('"p":\n')
p_min40 = generateRegion(lin_min40.theta, lin_min40.phi, reshape(Plist(:,:,1), 1, []));
p_min20 = generateRegion(lin_min20.theta, lin_min20.phi, reshape(Plist(:,:,2), 1, []));
p_0 = generateRegion(lin_0.theta, lin_0.phi, reshape(Plist(:,:,3), 1, []));
p_20 = generateRegion(lin_20.theta, lin_20.phi, reshape(Plist(:,:,4), 1, []));
p_40 = generateRegion(lin_40.theta, lin_40.phi, reshape(Plist(:,:,5), 1, []));
p_sched = createGainSchedule({p_min40 p_min20 p_0 p_20 p_40}, theta_dev, phi_dev);

% fprintf('"k":\n')
k_min40 = generateRegion(lin_min40.theta, lin_min40.phi, reshape(Kslist(:,:,1), 1, []));
k_min20 = generateRegion(lin_min20.theta, lin_min20.phi, reshape(Kslist(:,:,2), 1, []));
k_0 = generateRegion(lin_0.theta, lin_0.phi, reshape(Kslist(:,:,3), 1, []));
k_20 = generateRegion(lin_20.theta, lin_20.phi, reshape(Kslist(:,:,4), 1, []));
k_40 = generateRegion(lin_40.theta, lin_40.phi, reshape(Kslist(:,:,5), 1, []));
k_sched = createGainSchedule({k_min40 k_min20 k_0 k_20 k_40}, theta_dev, phi_dev);

% fprintf('"cp_long":\n')
cp_long_min20 = generateRegion(linp_min20.theta, linp_min20.phi, Cplist([1:4,9], 1));
cp_long_20 = generateRegion(linp_20.theta, linp_20.phi, Cplist([1:4,9], 3));
cp_long_sched = createGainSchedule({cp_long_min20 cp_long_20}, theta_dev, phi_dev);

% fprintf('"kp_long":\n')
kp_long_min20 = generateRegion(linp_min20.theta, linp_min20.phi, reshape(Krlonglist(:,:,1), 1, []));
kp_long_20 = generateRegion(linp_20.theta, linp_20.phi, reshape(Krlonglist(:,:,3), 1, []));
kp_long_sched = createGainSchedule({kp_long_min20 kp_long_20}, theta_dev, phi_dev);

% fprintf('"kp_lat":\n')
kp_lat_min20 = generateRegion(linp_min20.theta, linp_min20.phi, reshape(Krlatlist(:,:,1), 1, []));
kp_lat_20 = generateRegion(linp_20.theta, linp_20.phi, reshape(Krlatlist(:,:,3), 1, []));
kp_lat_sched = createGainSchedule({kp_lat_min20 kp_lat_20}, theta_dev, phi_dev);

% fprintf('"ssp":\n')
ssp_min20 = generateRegion(linp_min20.u, linp_min20.phi, ssplist(1:9, 1));
ssp_20 = generateRegion(linp_20.u, linp_20.phi, ssplist(1:9, 3));
ssp_sched = createGainSchedule({ssp_min20 ssp_20}, theta_dev, phi_dev);

% fprintf('"trimp":\n')
trimp_min20 = generateRegion(linp_min20.u, linp_min20.phi, trimplist(:,1));
trimp_20 = generateRegion(linp_20.u, linp_20.phi, trimplist(:,3));
trimp_sched = createGainSchedule({trimp_min20 trimp_20}, theta_dev, phi_dev);

gainScheduleJsonObjects = struct();
gainScheduleJsonObjects.Ad = a_sched;
gainScheduleJsonObjects.Bd = b_sched;
gainScheduleJsonObjects.C = c_sched;
gainScheduleJsonObjects.ss = ss_sched;
gainScheduleJsonObjects.trim = trim_sched;

gainScheduleJsonObjects.p = p_sched;
gainScheduleJsonObjects.k = k_sched;

gainScheduleJsonObjects.kp_long = kp_long_sched;
gainScheduleJsonObjects.kp_lat = kp_lat_sched;
gainScheduleJsonObjects.ssp = ssp_sched;
gainScheduleJsonObjects.trimp = trimp_sched;
gainScheduleJsonObjects.cp_long = cp_long_sched;


gainScheduleJsonObjects.r20_lat_path = "/home/seedship/TUM/SS21/Thesis/reachability/Rp20_lat.json";
gainScheduleJsonObjects.r20_long_path = "/home/seedship/TUM/SS21/Thesis/reachability/Rp20_long.json";
gainScheduleJsonObjects.rmin20_long_path = "/home/seedship/TUM/SS21/Thesis/reachability/Rpmin20_long.json";
gainScheduleJsonObjects.rmin20_lat_path = "/home/seedship/TUM/SS21/Thesis/reachability/Rpmin20_lat.json";

multisimplex = struct();
multisimplex.multi_simplex = gainScheduleJsonObjects;

simplexsupervisor = struct();
simplexsupervisor.simplex_supervisor = multisimplex;

json_gainSchedule = jsonencode(simplexsupervisor, 'PrettyPrint', true);
writeAsFile(json_gainSchedule, '/tmp/gainSchedule.json')

% reachabilityToJson(Rp0_20_lat, '/tmp/Rp20_lat.json')
% reachabilityToJson(Rp0_20_long, '/tmp/Rp20_long.json')
% reachabilityToJson(Rp0_min20_lat, '/tmp/Rpmin20_lat.json')
% reachabilityToJson(Rp0_min20_long, '/tmp/Rpmin20_long.json')



%% Saving to json

function setpoint = generateSetPoint(theta, phi)
    setpoint = struct('theta_rad', theta, 'phi_rad', phi);
end

function region =  generateRegion(theta, phi, k)
    region = struct('setpoint', generateSetPoint(theta, phi), 'k', reshape(k, 1, []));
end

function gainSchedule = createGainSchedule(regions, theta_dev, phi_dev)
    gainSchedule = struct();
    gainSchedule.regions = regions;
    gainSchedule.interpolation_stddev = generateSetPoint(theta_dev, phi_dev);
end

function writeAsFile(jsonData, filepath)
    fid = fopen(filepath, 'w');
    fprintf(fid, jsonData);
    fclose(fid);
end
