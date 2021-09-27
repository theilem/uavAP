function V = sample_ellipsoid(P, n, varArray)
    dim = size(P);
    
    if nargin < 3
       varArray = ones(dim(1), 1);
    end
    
    V = [];
    for i = 1:n
        v = rand(dim(1), 1) - 0.5;
        tmpVar = (-1) .^ (rand(dim(1), 1) > 0.5) .* (0.7 * rand(dim(1), 1) + 0.3) .* varArray;
        v = v .* tmpVar;
%         v = [0;0;0;rand(1) - 0.5; (90 * rand(1) + 10) * (rand(1) - 0.5)];
        s = v' * P * v;
        v = 1/sqrt(s) * v;
    V = [V;v'];
    end
end