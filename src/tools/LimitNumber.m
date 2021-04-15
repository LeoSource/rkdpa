function res = LimitNumber(low, x, up)
    if isscalar(x)
        if x>up
            res = up;
        elseif x<low
            res = low;
        else
            res = x;
        end
    else
        n = length(x);
        res = zeros(size(x));
        for idx=1:n
            if x(idx)>up(idx)
                res(idx) = up(idx);
            elseif x(idx)<low(idx)
                res(idx) = low(idx);
            else
                res(idx) = x(idx);
            end
        end
    end
end