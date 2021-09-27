function plotSaturation(LinearizationLimits, InitialEllipsoid, Kr, umax, umin)
%     x = state;
%     x(dim1) = sdpvar(1);
%     x(dim2) = sdpvar(1);

    x = [0;0;0;sdpvar(1);sdpvar(1)];
%     x = [sdpvar(1);0;0;sdpvar(1);0];
    figure(1)
    plot(LinearizationLimits.A * x <= LinearizationLimits.b, [], 'r')
    hold on
    plot(InitialEllipsoid.A * x <= InitialEllipsoid.b, [], 'b')
    
    figure(2)
    below = Polyhedron('A', -Kr, 'b', umin);
    below = intersect(below, LinearizationLimits);
    plot(below.A * x <= below.b, [], 'b')
    hold on
    in = Polyhedron('A', [Kr; -Kr], 'b', [-umin; umax]);
    in = intersect(in, LinearizationLimits);
    plot(in.A * x <= in.b, [], 'g')
    above = Polyhedron('A', Kr, 'b', -umax);
    above = intersect(above, LinearizationLimits);
    plot(above.A * x <= above.b, [], 'r')
    