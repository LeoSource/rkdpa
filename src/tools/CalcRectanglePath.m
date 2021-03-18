function via_pos = CalcRectanglePath(corner_pos, step)
% calculate the via position according to corner position of rectangle
% the sequence of corner position is like this
% 3--------4 
% |        |
% |        |
% 2--------1
% to do: extend other type

    numvp = 2*((corner_pos(3,4)-corner_pos(3,1))/step+1);
    for idx=1:numvp/2+1
        via_pos(3,2*idx-1) = corner_pos(3,1)+(idx-1)*step;
        via_pos(3,2*idx) = corner_pos(3,1)+(idx-1)*step;
        if mod(idx,2)==1
            via_pos(1,2*idx-1) = corner_pos(1,1);
            via_pos(1,2*idx) = corner_pos(1,2);
        else
            via_pos(1,2*idx-1) = corner_pos(1,2);
            via_pos(1,2*idx) = corner_pos(1,1);
        end
    end
    via_pos(2,:) = corner_pos(2,1);
end