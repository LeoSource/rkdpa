function via_pos = CalcRectanglePath(corner_pos, option)
% calculate the via position according to corner position of rectangle
% the sequence of corner position is like this
% 3--------4
% |        |
% |        |
% 2--------1
% option: path of 'm' or 's'

interval = 0.1;
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
cycle_num = floor(norm(step_vec1)/interval);
numvp = 2*cycle_num;
step = norm(step_vec1)/(cycle_num-1);
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

end