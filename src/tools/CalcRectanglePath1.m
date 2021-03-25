function via_pos = CalcRectanglePath1(corner_pos, step, option)
% calculate the via position according to corner position of rectangle
% the sequence of corner position is like this
% 3--------4 
% |        |
% |        |
% 2--------1
% option: path of 'm' or 's'
% to do: extend other type
    if strcmp(option, 's')
        step_vec1 = corner_pos(:,4)-corner_pos(:,1);
        step_vec2 = corner_pos(:,3)-corner_pos(:,2);
        start_pos1 = corner_pos(:,1);
        start_pos2 = corner_pos(:,2);
    else
        step_vec1 = corner_pos(:,2)-corner_pos(:,1);
        step_vec2 = corner_pos(:,3)-corner_pos(:,4);
        start_pos1 = corner_pos(:,1);
        start_pos2 = corner_pos(:,4);
    end
    numvp = 2*(round(norm(step_vec1)/step)+1);
    rest_dis = mod(norm(step_vec1), step);
    step_vec1 = step_vec1/norm(step_vec1);
    step_vec2 = step_vec2/norm(step_vec2);
    for idx=1:round(numvp/2)
        if mod(idx,2)==1
            via_pos(:,2*idx-1) = start_pos1+step_vec1*step*(idx-1);
            via_pos(:,2*idx) = start_pos2+step_vec2*step*(idx-1);
        else
            via_pos(:,2*idx-1) = start_pos2+step_vec2*step*(idx-1);
            via_pos(:,2*idx) = start_pos1+step_vec1*step*(idx-1);
        end
    end
    % to make sure that the distance is greater than double value of arc radius
    if rest_dis>2*0.04
        if mod(idx,2)==1
            via_pos(:,end+1) = corner_pos(:,3);
            via_pos(:,end+1) = corner_pos(:,2);
        else
            via_pos(:,end+1) = corner_pos(:,2);
            via_pos(:,end+1) = corner_pos(:,3);
        end
    end
end