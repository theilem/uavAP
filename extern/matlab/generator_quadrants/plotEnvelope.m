function toPlot = plotEnvelope(poly, dim1, dim2, state)


    lD = min(dim1, dim2);
    uD = max(dim1, dim2);
    A = poly.A;
    b = poly.b;
    dimA = size(A);
    if nargin<4 || isempty(state)
        state = zeros(dimA(2),1);
    end
    v = sdpvar(dimA(2),1);
    v(1:lD - 1) = state(1:lD - 1);
    v(lD+1:uD - 1) = state(lD+1:uD - 1);
    v(uD + 1:end) = state(uD + 1:end);
    
    toPlot = Polyhedron(lmi(A * v <= b));
    
    plot(A * v <= b);
end