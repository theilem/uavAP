function saveAB
%     coef_20 = regression20();
%     coef_25 = regression25();
%     lin20_min40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_-40/lin.json');
%     lin20_min20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_-20/lin.json');
%     lin20_0 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_0/lin.json');
%     lin20_20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_20/lin.json');
%     lin20_40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_40/lin.json');
% 
%     lin25_min40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_-40/lin.json');
%     lin25_min20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_-20/lin.json');
%     lin25_0 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_0/lin.json');
%     lin25_20 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_20/lin.json');
%     lin25_40 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/25_40/lin.json');
% 
%     u_dev = 2;
%     phi_dev = 0.1;
% 
%     [A20_min40, B20_min40, ss20_min40, trim20_min40] = createSysAB(coef_20, lin20_min40);
%     [A20_min20, B20_min20, ss20_min20, trim20_min20] = createSysAB(coef_20, lin20_min20);
%     [A20_0, B20_0, ss20_0, trim20_0] = createSysAB(coef_20, lin20_0);
%     [A20_20, B20_20, ss20_20, trim20_20] = createSysAB(coef_20, lin20_20);
%     [A20_40, B20_40, ss20_40, trim20_40] = createSysAB(coef_20, lin20_40);
% 
%     [A25_min40, B25_min40, ss25_min40, trim25_min40] = createSysAB(coef_25, lin25_min40);
%     [A25_min20, B25_min20, ss25_min20, trim25_min20] = createSysAB(coef_25, lin25_min20);
%     [A25_0, B25_0, ss25_0, trim25_0] = createSysAB(coef_25, lin25_0);
%     [A25_20, B25_20, ss25_20, trim25_20] = createSysAB(coef_25, lin25_20);
%     [A25_40, B25_40, ss25_40, trim25_40] = createSysAB(coef_25, lin25_40);
%     
%     % fprintf('"A":\n')
%     A20_min40 = generateRegion(lin20_min40.u, lin20_min40.phi, A20_min40);
%     A20_min20 = generateRegion(lin20_min20.u, lin20_min20.phi, A20_min20);
%     A20_0 = generateRegion(lin20_0.u, lin20_0.phi, A20_0);
%     A20_20 = generateRegion(lin20_20.u, lin20_20.phi, A20_20);
%     A20_40 = generateRegion(lin20_40.u, lin20_40.phi, A20_40);
%     A25_min40 = generateRegion(lin25_min40.u, lin25_min40.phi, A25_min40);
%     A25_min20 = generateRegion(lin25_min20.u, lin25_min20.phi, A25_min20);
%     A25_0 = generateRegion(lin25_0.u, lin25_0.phi, A25_0);
%     A25_20 = generateRegion(lin25_20.u, lin25_20.phi, A25_20);
%     A25_40 = generateRegion(lin25_40.u, lin25_40.phi, A25_40);
%     a_sched = createGainSchedule({A20_min40 A20_min20 A20_0 A20_20 A20_40 ...
%         A25_min40 A25_min20 A25_0 A25_20 A25_40}, u_dev, phi_dev);
%     
%     % fprintf('"B":\n')
%     B20_min40 = generateRegion(lin20_min40.u, lin20_min40.phi, B20_min40);
%     B20_min20 = generateRegion(lin20_min20.u, lin20_min20.phi, B20_min20);
%     B20_0 = generateRegion(lin20_0.u, lin20_0.phi, B20_0);
%     B20_20 = generateRegion(lin20_20.u, lin20_20.phi, B20_20);
%     B20_40 = generateRegion(lin20_40.u, lin20_40.phi, B20_40);
%     B25_min40 = generateRegion(lin25_min40.u, lin25_min40.phi, B25_min40);
%     B25_min20 = generateRegion(lin25_min20.u, lin25_min20.phi, B25_min20);
%     B25_0 = generateRegion(lin25_0.u, lin25_0.phi, B25_0);
%     B25_20 = generateRegion(lin25_20.u, lin25_20.phi, B25_20);
%     B25_40 = generateRegion(lin25_40.u, lin25_40.phi, B25_40);
%     b_sched = createGainSchedule({B20_min40 B20_min20 B20_0 B20_20 B20_40 ...
%         B25_min40 B25_min20 B25_0 B25_20 B25_40}, u_dev, phi_dev);
%     
%     % fprintf('"ss":\n')
%     ss20_min40 = generateRegion(lin20_min40.u, lin20_min40.phi, ss20_min40);
%     ss20_min20 = generateRegion(lin20_min20.u, lin20_min20.phi, ss20_min20);
%     ss20_0 = generateRegion(lin20_0.u, lin20_0.phi, ss20_0);
%     ss20_20 = generateRegion(lin20_20.u, lin20_20.phi, ss20_20);
%     ss20_40 = generateRegion(lin20_40.u, lin20_40.phi, ss20_40);
%     ss25_min40 = generateRegion(lin25_min40.u, lin25_min40.phi, ss25_min40);
%     ss25_min20 = generateRegion(lin25_min20.u, lin25_min20.phi, ss25_min20);
%     ss25_0 = generateRegion(lin25_0.u, lin25_0.phi, ss25_0);
%     ss25_20 = generateRegion(lin25_20.u, lin25_20.phi, ss25_20);
%     ss25_40 = generateRegion(lin25_40.u, lin25_40.phi, ss25_40);
%     ss_sched = createGainSchedule({ss20_min40 ss20_min20 ss20_0 ss20_20 ss20_40 ...
%         ss25_min40 ss25_min20 ss25_0 ss25_20 ss25_40}, u_dev, phi_dev);
%     
%     % fprintf('"trim":\n')
%     trim20_min40 = generateRegion(lin20_min40.u, lin20_min40.phi, trim20_min40);
%     trim20_min20 = generateRegion(lin20_min20.u, lin20_min20.phi, trim20_min20);
%     trim20_0 = generateRegion(lin20_0.u, lin20_0.phi, trim20_0);
%     trim20_20 = generateRegion(lin20_20.u, lin20_20.phi, trim20_20);
%     trim20_40 = generateRegion(lin20_40.u, lin20_40.phi, trim20_40);
%     trim25_min40 = generateRegion(lin25_min40.u, lin25_min40.phi, trim25_min40);
%     trim25_min20 = generateRegion(lin25_min20.u, lin25_min20.phi, trim25_min20);
%     trim25_0 = generateRegion(lin25_0.u, lin25_0.phi, trim25_0);
%     trim25_20 = generateRegion(lin25_20.u, lin25_20.phi, trim25_20);
%     trim25_40 = generateRegion(lin25_40.u, lin25_40.phi, trim25_40);
%     trim_sched = createGainSchedule({trim20_min40 trim20_min20 trim20_0 trim20_20 trim20_40 ...
%         trim25_min40 trim25_min20 trim25_0 trim25_20 trim25_40}, u_dev, phi_dev);
%     
%     % fprintf('"c":\n')
% %     c20_min40 = generateRegion(lin20_min40.u, lin20_min40.phi, calcC(lin20_min40));
% %     c20_min20 = generateRegion(lin20_min20.u, lin20_min20.phi, calcC(lin20_min40));
% %     c20_0 = generateRegion(lin20_0.u, lin20_0.phi, calcC(lin20_0));
% %     c20_20 = generateRegion(lin20_20.u, lin20_20.phi, calcC(lin20_20));
% %     c20_40 = generateRegion(lin20_40.u, lin20_40.phi, calcC(lin20_40));
% %     c25_min40 = generateRegion(lin25_min40.u, lin25_min40.phi, calcC(lin25_min40));
% %     c25_min20 = generateRegion(lin25_min20.u, lin25_min20.phi, calcC(lin25_min20));
% %     c25_0 = generateRegion(lin25_0.u, lin25_0.phi, calcC(lin25_0));
% %     c25_20 = generateRegion(lin25_20.u, lin25_20.phi, calcC(lin25_20));
% %     c25_40 = generateRegion(lin25_40.u, lin25_40.phi, calcC(lin25_40));
% %     c_sched = createGainSchedule({c20_min40 c20_min20 c20_0 c20_20 c20_40 ...
% %         c25_min40 c25_min20 c25_0 c25_20 c25_40}, u_dev, phi_dev);
% 
%     gainScheduleJsonObjects = struct();
%     gainScheduleJsonObjects.A = a_sched;
%     gainScheduleJsonObjects.B = b_sched;
%     gainScheduleJsonObjects.ss = ss_sched;
%     gainScheduleJsonObjects.trim = trim_sched;
% %     gainScheduleJsonObjects.c = c_sched;
% 
%     json_gainSchedule = jsonencode(gainScheduleJsonObjects, 'PrettyPrint', true);
%     writeAsFile(json_gainSchedule, '/tmp/ABinfo.json')
    
%%% SaveABOne
    coef_20 = regression20();
    lin20_0 = loadJson('/home/seedship/TUM/SS21/Thesis/multi-SI/linearization/20_0/lin.json');

    u_dev = 2;
    phi_dev = 0.1;

    [A20_0, B20_0, ss20_0, trim20_0] = createSysAB(coef_20, lin20_0);
    
    % fprintf('"A":\n')
    A20_0 = generateRegion(lin20_0.u, lin20_0.phi, A20_0);
    a_sched = createGainSchedule({A20_0}, u_dev, phi_dev);
    
    % fprintf('"B":\n')
    B20_0 = generateRegion(lin20_0.u, lin20_0.phi, B20_0);
    b_sched = createGainSchedule({B20_0}, u_dev, phi_dev);
    
    % fprintf('"ss":\n')
    ss20_0 = generateRegion(lin20_0.u, lin20_0.phi, ss20_0);
    ss_sched = createGainSchedule({ss20_0}, u_dev, phi_dev);
    
    % fprintf('"trim":\n')
    trim20_0 = generateRegion(lin20_0.u, lin20_0.phi, trim20_0);
    trim_sched = createGainSchedule({trim20_0}, u_dev, phi_dev);
    
    gainScheduleJsonObjects = struct();
    gainScheduleJsonObjects.A = a_sched;
    gainScheduleJsonObjects.B = b_sched;
    gainScheduleJsonObjects.ss = ss_sched;
    gainScheduleJsonObjects.trim = trim_sched;

    json_gainSchedule = jsonencode(gainScheduleJsonObjects, 'PrettyPrint', true);
    writeAsFile(json_gainSchedule, '/tmp/ABinfoOne.json')
end


function [A, B, ss, trim] = createSysAB(coef, lin)
    [Along, Along_corr, Alat_corr, Alat, Blong, Blong_corr, Blat_corr, Blat, ~, ~, ~, ~, ~, ~, ~, ~] ...
        = generate_submatricies(coef, lin);
    A = [Along, Along_corr; Alat_corr, Alat];
    B = [Blong, Blong_corr; Blat_corr, Blat];
    ss = [lin.u; lin.w; lin.q; lin.theta; lin.v; lin.p; lin.r; lin.phi];
    trim = [lin.pitch_ctrl; lin.throttle_ctrl; lin.roll_ctrl; lin.yaw_ctrl];
end

% function c = calcC(lin)
%     c = [lin.u_dot, lin.w_dot, lin.q_dot, lin.v_dot, lin.p_dot, lin.r_dot];
% end


%% Saving to json

function setpoint = generateSetPoint(u, phi)
    setpoint = struct('u', u, 'phi_rad', phi);
end

function region =  generateRegion(u, phi, k)
    region = struct('setpoint', generateSetPoint(u, phi), 'k', k); %reshape(k, 1, [])
end

function gainSchedule = createGainSchedule(regions, u_dev, phi_dev)
    gainSchedule = struct();
    gainSchedule.regions = regions;
    gainSchedule.interpolation_stddev = generateSetPoint(u_dev, phi_dev);
end

function writeAsFile(jsonData, filepath)
    fid = fopen(filepath, 'w');
    fprintf(fid, jsonData);
    fclose(fid);
end
