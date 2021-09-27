function ret = polyContains(polyA, polyb, point)
    ret = 0;
    shape = size(polyA);
    for idx = 1:shape(3)
        if min(polyA(:,:,idx) * point <= polyb(:,idx))
            ret = 1;
            return
        end
    end
end