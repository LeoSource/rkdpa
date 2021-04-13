function res = LimitNumber(low, x, up)
    if x>up
        res = up;
    elseif x<low
        res = low;
    else
        res = x;
    end
end