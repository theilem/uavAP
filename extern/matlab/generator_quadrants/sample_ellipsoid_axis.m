function V = sample_ellipsoid_axis(P)
    dim = size(P);
    V = zeros(2 * dim(1), dim(1));
    for i = 1:2 * dim(1)
        v = zeros(dim(1), 1);
        v(ceil(i/2)) = (-1)^i;
        s = v' * P * v;
        v = 1/sqrt(s) * v;
        V(i,:) = v;
    end
end