function plotReachabilityEvolution(polys, dim1, dim2, state)
    hold on
    
    if nargin<4 || isempty(state)
        A = polys(1).A;
        dimA = size(A);
        state = zeros(dimA(2),1);
    end
    
    for idx = 1:length(polys)
        fprintf('Processed idx %i of %i\n', idx, length(polys))
        plotEnvelope(polys(idx), dim1, dim2, state);
    end

end