function [Kr, Rp, ActuationLimits, LinearizationLimits, Limits, InitialEllipsoid, tu] = pitch_reachability_lat(lin, lin_p, reg_coefs, P, phi_min, phi_max, N)

[~, ~, ~, Alat, ~, ~, ~, Blat, ~, ~, ~, ~, ~, lat0, ~, ~] ...
        = generate_submatricies(reg_coefs, lin);
    
    F = 100; % AlVolo/uavEE update interval
    continuous = ss(Alat, Blat, eye(4), zeros(4, 2));
    discrete = c2d(continuous, 1/F);
    Ad = discrete.A;
    Bd = discrete.B;

%     Q = 0.001 * eye(4);
    Q = zeros(4);
    Q(1,1) = 1;
    Q(4,4) = 1;
    R = eye(2);

    Kr = lqrd(Alat, Blat, Q, R, 1/F);

    tx = [lin.v - lin_p.v;
          lin.p - lin_p.p;
          lin.r - lin_p.r;
          lin.phi - lin_p.phi];

    tu = [lin.roll_ctrl;
          lin.yaw_ctrl];

    u = sdpvar(2, 1);

%     Vp = sample_ellipsoid(P, 18);
    P(9,:) = [];
    P(:,9) = [];
    P(1:4,:) = [];
    P(:,1:4) = [];
    Vp = sample_ellipsoid_axis(P);

    Abound = zeros(2, 4);
    Abound(1:2,4) = [-1;1];

    Bbound = [-phi_min;
               phi_max];

    LinearizationLimits = Polyhedron('A', Abound, 'b', Bbound);
    x = sdpvar(4,1);
%     Fu = [-1 - tu <= Kr * x;
%           Kr * x <= 1 - tu];
%     ActuationLimits = Polyhedron(Fu);
    ActuationLimits = Polyhedron('A', [-Kr; Kr], 'b', [1 - tu; 1 + tu]);
    Limits = intersect(ActuationLimits, LinearizationLimits);

    InitialEllipsoid = Polyhedron('v', Vp) - tx';
    fprintf('Calculated H Rep Polyhedron\n')
    InitialEllipsoid.minHRep();
%     InitialEllipsoid.minVRep();
    fprintf('Calculated Min H Rep Polyhedron\n')
    Sinitial = intersect(InitialEllipsoid, Limits);
    fprintf('Intersected with Bound\n')
    Sinitial.minHRep()
%     Sinitial.minVRep()
    fprintf('Calculated Min Rep\n')

    Rp = [Sinitial];
    lastreachability = copy(Sinitial);
    for i = 1:N
        fprintf('Iteration %i of %i\n', i, N)
        lastA = lastreachability.A;
        lastb = lastreachability.b;
        lastAe = lastreachability.Ae;
        lastbe = lastreachability.be;
        
        ABk = Ad - Bd * Kr;
        newA = lastA * ABk;
        newAe = lastAe * ABk;
        
        newReachability = Polyhedron('A', newA, 'b', lastb, 'Ae', newAe, 'be', lastbe) - lat0/F;
        newReachability = intersect(newReachability, Limits);
        newReachability.minHRep()
        
%         valid = newReachability <= Limits;
%         while ~valid
%             fprintf('Limits broken at itnum %i, recalculating and not minimizing\n', i)
%             newReachability = intersect(newReachability, Limits);
%             valid = newReachability <= Limits;
%             fprintf('New reachability validty: %d\n', valid)
%         end
%         
%         empty = newReachability.isEmptySet;
%         fprintf('newReachability Empty? %i\n', empty)
%         subLast = newReachability <= lastreachability;
%         fprintf('newReachability <= lastreachability? %i\n', subLast)
%         subInitial = newReachability <= Sinitial;
%         fprintf('newReachability <= initial? %i\n', subInitial)
%         if empty || subLast || subInitial
%             return
%         end
        Rp = [Rp, copy(newReachability)];
        lastreachability = copy(newReachability);
    end
end