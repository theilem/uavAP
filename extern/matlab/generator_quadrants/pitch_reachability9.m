function [Ad, Bd, Kr, Rp, ActuationLimits, LinearizationLimits, Limits, InitialEllipsoid] = pitch_reachability9(lin, lin_p, reg_coefs, P, u_min, u_max, theta_min, theta_max, phi_min, phi_max, h_min, h_max, N)

[Along, Along_corr, Alat_corr, Alat, Blong, Blong_corr, Blat_corr, Blat, Hlong, Hlat, ~, ~, long0, lat0, H0, ~] ...
        = generate_submatricies(reg_coefs, lin);
    A = [Along, Along_corr, zeros(4, 1);
         Alat_corr, Alat, zeros(4,1);
         Hlong, Hlat, 0];
    B = [Blong, Blong_corr;
         Blat_corr, Blat;
         zeros(1, 4)];
    
    constants = [long0; lat0; H0];

    Q = 0.001 * eye(9);
    Q(1,1) = 1;
    Q(4,4) = 1;
    Q(8,8) = 1;
    R = eye(4);

    F = 100; % AlVolo/uavEE update interval
    Kr = lqrd(A, B, Q, R, 1/F);

    continuous = ss(A, B, eye(9), zeros(9, 4));
    discrete = c2d(continuous, 1/F);
    Ad = discrete.A;
    Bd = discrete.B;

    tx = [lin.u - lin_p.u;
          lin.w - lin_p.w;
          lin.q - lin_p.q;
          lin.theta - lin_p.theta;
          lin.v - lin_p.v;
          lin.p - lin_p.p;
          lin.r - lin_p.r;
          lin.phi - lin_p.phi;
          0];

    tu = [lin.pitch_ctrl;
          lin.throttle_ctrl;
          lin.roll_ctrl;
          lin.yaw_ctrl;];

    u = sdpvar(4, 1);

%     Vp = sample_ellipsoid(P, 18);
%     P(5:8,:) = [];
%     P(:,5:8) = [];
    Vp = sample_ellipsoid_axis(P);

    Abound = zeros(6, 5);
    Abound(1:2,1) = [-1;1];
    Abound(3:4,4) = [-1;1];
    Abound(5:6,8) = [-1;1];
    Abound(7:8,9) = [-1;1];

    Bbound = [-u_min;
               u_max;
              -theta_min;
               theta_max;
               -phi_min;
               phi_max
              -h_min;
               h_max ];

    LinearizationLimits = Polyhedron('A', Abound, 'b', Bbound);
    x = sdpvar(9,1);
    Fu = [-1 - tu <= Kr * x;
          Kr * x <= 1 - tu];
    ActuationLimits = Polyhedron(Fu);
    Limits = intersect(ActuationLimits, LinearizationLimits);
    Limits.minHRep();
    Limits.minVRep();

    InitialEllipsoid = Polyhedron('v', Vp) - tx';
    fprintf('Calculated H Rep Polyhedron\n')
    InitialEllipsoid.minHRep();
    fprintf('Calculated Min H Rep Polyhedron\n')
    Sinitial = intersect(InitialEllipsoid, Limits);
    fprintf('Intersected with Bound\n')
    Sinitial.minHRep()
    Sinitial.minVRep();
    fprintf('Calculated Min H Rep\n')

    Rp = [Sinitial];
    lastreachability = Sinitial;

    ABk = Ad - Bd * Kr;
    for i = 1:N
        fprintf('Iteration %i of %i\n', i, N)
        lastA = lastreachability.A;
        lastb = lastreachability.b;
        lastAe = lastreachability.Ae;
        
        newA = lastA * ABk;
        
%         newReachability = Polyhedron('A', newA, 'b', lastb, 'Ae', newAe, 'be', lastbe) - constants/F;
%         newReachability_temp = Polyhedron('A', newA, 'b', lastb);
%         newReachability_temp<=newReachability_temp
        newReachability = Polyhedron('A', newA, 'b', lastb) - constants/F;
        newReachability = intersect(newReachability, Limits);
        

        newReachability.minHRep()
%         newReachability.computeVRep()
        
        valid = newReachability <= Limits;
        if ~valid
            fprintf('Limits broken at itnum %i, recalculating and not minimizing\n', i)
            newReachability = intersect(newReachability, Limits);
%             newReachability.R
            valid = newReachability <= Limits;
        end
        
        
        
        empty = newReachability.isEmptySet;
        fprintf('newReachability Empty? %i\n', empty)
        subLast = newReachability <= lastreachability;
        fprintf('newReachability <= lastreachability? %i\n', subLast)
        subInitial = newReachability <= Sinitial;
        fprintf('newReachability <= initial? %i\n', subInitial)
        if empty || subLast || subInitial
            return
        end
        Rp = [Rp, copy(newReachability)];
        lastreachability = copy(newReachability);
    end
end