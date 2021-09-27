x_abs_next = out.ScopeData.signals(4).values;
% x_abs = out.ScopeData.signals(1).values;

usp = x_abs_next(:,1) - ssplist(1, 3);
wsp = x_abs_next(:,2) - ssplist(2, 3);
qsp = x_abs_next(:,3) - ssplist(3, 3);
thetasp = x_abs_next(:,4) - ssplist(4, 3);
hsp = x_abs_next(:,9) - ssplist(9, 3);

us = x_abs_next(:,1) - sslist(1, 3);
ws = x_abs_next(:,2) - sslist(2, 3);
qs = x_abs_next(:,3) - sslist(3, 3);
thetas = x_abs_next(:,4) - sslist(4, 3);
vs = x_abs_next(:,5) - sslist(5, 3);
ps = x_abs_next(:,6) - sslist(6, 3);
rs = x_abs_next(:,7) - sslist(7, 3);
phis = x_abs_next(:,8) - sslist(8, 3);
hs = x_abs_next(:,9) - sslist(9, 3);

xsim = [us, ws, qs, thetas, vs, ps, rs, phis, hs];

sContains = [];
rContainsLong = [];
rContainsLat = [];

safetyCtrl = out.ScopeData.signals(11).values;

for idx = 1:length(out.tout)
    sContains = [sContains, xsim(idx,:) * Plist(:,:,3) * xsim(idx,:)' <= 1];
    
    xr = [usp, wsp, qsp, thetasp, hsp];
    rContainsLong = [rContainsLong, polyContains(long20_A, long20_b, xr(idx,:)')];
end
