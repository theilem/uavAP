function [Kr, Rp, ActuationLimits, LinearizationLimits, Limits, InitialEllipsoid, tu] = pitch_reachability_long(lin, lin_p, reg_coefs, P, u_min, u_max, theta_min, theta_max, h_min, h_max, N)

    [Along, ~, ~, ~, Blong, ~, ~, ~, Hlong, ~, ~, ~, long0, ~, H0, ~] ...
        = generate_submatricies(reg_coefs, lin);
    As = [Along, zeros(4,1); Hlong, 0];
    Bs = [Blong; zeros(1,2)];

    F = 100; % AlVolo/uavEE update interval
    continuous = ss(As, Bs, eye(5), zeros(5, 2));
    discrete = c2d(continuous, 1/F);
    Ad = discrete.A;
    Bd = discrete.B;
    
    constants = [long0; H0];

    Q = 0.001 * eye(4);
    Q = zeros(4);
    Q(1,1) = 1;
    Q(4,4) = 10;
    R = eye(2);

    Kr = lqrd(Along, Blong, Q, R, 1/F);
    Kr = [Kr, [0; 0]];

    tx = [lin.u - lin_p.u;
          lin.w - lin_p.w;
          lin.q - lin_p.q;
          lin.theta - lin_p.theta;
          0];

    tu = [lin.pitch_ctrl;
          lin.throttle_ctrl;];

    u = sdpvar(2, 1);

%     Vp = sample_ellipsoid(P, 18);
    P(5:8,:) = [];
    P(:,5:8) = [];
    Vp = sample_ellipsoid_axis(P);
%     varArray = [Vp(2,1); Vp(4,2); Vp(6,3); Vp(8,4); Vp(10,5)];
    varArray = [Vp(2,1); 0;0; Vp(8,4); Vp(10,5)];
    varArray
    Vpextra = sample_ellipsoid(P, 25, varArray);
    Vpextra - tx'
    Vp = [Vp; Vpextra];

    Abound = zeros(6, 5);
    Abound(1:2,1) = [-1;1];
    Abound(3:4,4) = [-1;1];
    Abound(5:6,5) = [-1;1];

    Bbound = [-u_min;
               u_max;
              -theta_min;
               theta_max;
              -h_min;
               h_max ];

    LinearizationLimits = Polyhedron('A', Abound, 'b', Bbound);
    ActuationLimits = Polyhedron('A', [-Kr; Kr], 'b', [1 - tu; 1 + tu]);
    Limits = intersect(ActuationLimits, LinearizationLimits);

    InitialEllipsoid = Polyhedron('v', Vp) - tx';
    fprintf('Calculated H Rep Polyhedron\n')
    InitialEllipsoid.minHRep();
    InitialEllipsoid.minVRep();
    fprintf('Calculated Min H Rep Polyhedron\n')
    Sinitial = intersect(InitialEllipsoid, Limits);
    fprintf('Intersected with Bound\n')
    Sinitial.minHRep()
    Sinitial.minVRep()
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
        
        newReachability = Polyhedron('A', newA, 'b', lastb, 'Ae', newAe, 'be', lastbe) - constants/F;
        newReachability = intersect(newReachability, Limits);
        newReachability.minHRep()

        if length(newReachability.b) >= 1000
            fprintf('More than 1000 inequalities, breaking\n')
            return
        end
        Rp = [Rp, copy(newReachability)];
        lastreachability = copy(newReachability);
    end
end