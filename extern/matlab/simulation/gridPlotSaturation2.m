function gridPlotSaturation2(InitialEllipsoid, Kr, tu, xDim, yDim, xmin, xmax, ymin, ymax, state)

    if nargin < 10
        Ashape = size(InitialEllipsoid.A);
        state = zeros(Ashape(2), 1);
    end
    
    figure(1)
    % Limits (universal set over limits)
%     gridPlotReachability(Polyhedron('A', zeros(1,2), 'b', 0), 1, 2, xmin, xmax, ymin, ymax)
    x = sdpvar(2,1);
    plot([xmin; ymin] <= x <= [xmax; ymax], x, 'r')
    hold on
    % InitialEllipsoid
    gridPlotReachability(InitialEllipsoid, xDim, yDim, xmin, xmax, ymin, ymax, 'b', 1, state);
    
    figure(2)
    hold on
    umin = -1 - tu;
    umax = 1 - tu;
    
    plotSingletonLimitSet(Kr(1,:), tu(1), xDim, yDim, xmin, xmax, ymin, ymax, state, 'y', 'g', 'k', 0.5);
    plotSingletonLimitSet(Kr(2,:), tu(2), xDim, yDim, xmin, xmax, ymin, ymax, state, 'b', 'g', 'w', 0.5);
end
    
function plotSingletonLimitSet(Kr, tu, xDim, yDim, xmin, xmax, ymin, ymax, state, colorb, colori, colora, alpha)
    umin = -1 - tu;
    umax = 1 - tu;
    below = Polyhedron('A', -Kr, 'b', umin);
    gridPlotReachability(below, xDim, yDim, xmin, xmax, ymin, ymax, colorb, alpha, state);
    in = Polyhedron('A', [Kr; -Kr], 'b', [-umin; umax]);
    gridPlotReachability(in, xDim, yDim, xmin, xmax, ymin, ymax, colori, alpha, state);
    above = Polyhedron('A', Kr, 'b', -umax);
    gridPlotReachability(above, xDim, yDim, xmin, xmax, ymin, ymax, colora, alpha, state);
end