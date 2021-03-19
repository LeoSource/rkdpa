function via_pos = CalcRectanglePath(corner_pos, step, direction)
% calculate the via position according to corner position of rectangle
% the sequence of corner position is like this
% 3--------4 
% |        |
% |        |
% 2--------1
% direction: 0 for static, -1 for cycle motion, 1 for step motion
% to do: extend other type

    cycle_idx = find(direction==-1);
    step_idx = find(direction==1);
    static_idx = find(direction==0);
    numvp = 2*((corner_pos(step_idx,4)-corner_pos(step_idx,1))/step+1);
    for idx=1:round(numvp/2)
        via_pos(step_idx,2*idx-1) = corner_pos(step_idx,1)+(idx-1)*step;
        via_pos(step_idx,2*idx) = corner_pos(step_idx,1)+(idx-1)*step;
        if mod(idx,2)==1
            via_pos(cycle_idx,2*idx-1) = corner_pos(cycle_idx,1);
            via_pos(cycle_idx,2*idx) = corner_pos(cycle_idx,2);
        else
            via_pos(cycle_idx,2*idx-1) = corner_pos(cycle_idx,2);
            via_pos(cycle_idx,2*idx) = corner_pos(cycle_idx,1);
        end
    end
    via_pos(static_idx,:) = corner_pos(static_idx,1);
end