function [Ad, Bd, Kr, Rp, ActuationLimits, LinearizationLimits, Limits] = pitch_reachability2(lin, lin_p, reg_coefs, P, u_min, u_max, theta_min, theta_max, h_min, h_max, N)

    [Along, ~, ~, ~, Blong, ~, ~, ~, Hlong, ~, ~, ~, long0, ~, H0, ~] ...
        = generate_submatricies(reg_coefs, lin);
    A = [Along, zeros(4,1); Hlong, 0];
    B = [Blong; zeros(1, 2)];
    
    constants = [long0; H0];

    Q = 0.001 * eye(5);
    Q(1,1) = 1;
    Q(4,4) = 1;
    R = eye(2);

    F = 100; % AlVolo/uavEE update interval
    Kr = lqrd(A, B, Q, R, 1/F);

    continuous = ss(A, B, eye(5), zeros(5, 2));
    discrete = c2d(continuous, 1/F);
    Ad = discrete.A;
    Bd = discrete.B;

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
    x = sdpvar(5,1);
    Fu = [-1 - tu <= Kr * x;
          Kr * x <= 1 - tu];
    ActuationLimits = Polyhedron(Fu);
    Limits = intersect(ActuationLimits, LinearizationLimits);

    InitialEllipsoid = Polyhedron('v', Vp) - tx';
    fprintf('Calculated H Rep Polyhedron\n')
    InitialEllipsoid.minHRep();
    fprintf('Calculated Min H Rep Polyhedron\n')
    Sinitial = intersect(InitialEllipsoid, Limits);
    fprintf('Intersected with Bound\n')
    Sinitial.minHRep()
    fprintf('Calculated Min H Rep\n')

    Rp = [Sinitial];
    lastreachability = Sinitial;
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
        
        valid = newReachability <= Limits;
        while ~valid
            fprintf('Limits broken, recalculating\n')
            newReachability = intersect(newReachability, Limits);
            newReachability.minHRep();
            valid = newReachability <= Limits;
        end
        
        fprintf('newReachability Empty? %i\n', newReachability.isEmptySet)
        fprintf('newReachability <= lastreachability? %i\n', newReachability <= lastreachability)
        fprintf('newReachability <= initial? %i\n', newReachability <= Sinitial)
        if newReachability.isEmptySet || newReachability <= lastreachability || newReachability <= Sinitial
            return
        end
        Rp = [Rp, newReachability];
        lastreachability = newReachability;
    end
end