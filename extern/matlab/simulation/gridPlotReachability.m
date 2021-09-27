function [xs, ys] = gridPlotReachability(polyArray, dimX, dimY, minX, maxX, minY, maxY, color, alpha, state)
    numX = 100;
    numY = 100;
    
    
    if nargin < 10
        Ashape = size(polyArray(1).A);
        state = zeros(Ashape(2), 1);
    end
    if nargin < 8
        color = 'b';
    end
    if nargin < 9
        alpha = 0.5;
    end
    
    xs = [];
    ys = [];
    
    xList = linspace(minX, maxX, numX);
    yList = linspace(minY, maxY, numY);
    
    for x = xList
        state(dimX) = x;
        for y = yList
            state(dimY) = y;
            if max(polyArray.contains(state))
                xs = [xs, x];
                ys = [ys, y];
            end
        end
    end
    scatter(xs, ys, color, 's', 'filled', 'MarkerFaceAlpha', alpha,'MarkerEdgeAlpha', alpha)
%     scatter(xs, ys, 'r', 'o', 'filled', 'MarkerFaceAlpha',.2,'MarkerEdgeAlpha',.2)
end

% function contains = polyContainsArray(polyArray, point)
%     contains = 0;
%     for idx = length(polyArray)
%        if polyArray(idx).contains(point)
%            contains = 1;
%            return;
%        end
%     end
% end