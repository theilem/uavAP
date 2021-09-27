function [A, b] = polyArrayToTensor(poly)
    len = length(poly(1).b);
    for idx = 2:length(poly)
        if length(poly(idx).b) > len
           len = length(poly(idx).b);
        end
    end
    
    for idx = 1:length(poly)
        tmpA = poly(idx).A;
        tmpb = poly(idx).b;
        if length(poly(idx).A) < len
            tmpA(len,:) = 0;
            tmpb(len) = 0;
        end
        A(:,:,idx) = tmpA;
        b(:,idx) = tmpb;
    end
end