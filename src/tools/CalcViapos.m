function via_posrpy = CalcViapos(vision_pos, type)

    npos = size(vision_pos,2);
    if strcmp(type, 'toilet')
        center = mean(vision_pos, 2);
        rot_center = [0, 1, 0; 1, 0, 0; 0, 0, -1];
        rpy = tr2rpy(rot_center, 'xyz');
        via_posrpy(:,1) = [center; rpy'];
        theta = 20*pi/180;%angle of vertical plane
        for idx=1:npos
            pos_tmp = vision_pos(:,idx);
            len = norm(center-pos_tmp);
            pos_high = center;
            pos_high(3) = center(3)+len*cot(theta);
            z0 = pos_tmp-pos_high;
            z0 = z0/norm(z0);
            vec_high_center = center-pos_high;
            x0 = cross(vec_high_center, z0);
            x0 = x0/norm(x0);
            y0 = cross(z0,x0);
            rot_mat = [x0, y0, z0];
            rpy = tr2rpy(rot_mat, 'xyz');
            via_posrpy(:,idx+1) = [pos_tmp; rpy'];
        end
    elseif strcmp(type,'table')
        theta = 40*pi/180;%angle of horizontal plane
        origin = 0.5*(vision_pos(:,1)+vision_pos(:,2));
        len = norm(origin-vision_pos(:,3));
        origin(3) = origin(3)+len*tan(theta);
        interval = 0.02;
        step_vec1 = vision_pos(:,3)-vision_pos(:,4);
        step_vec2 = vision_pos(:,2)-vision_pos(:,1);
        start_pos1 = vision_pos(:,4);
        start_pos2 = vision_pos(:,1);
        cycle_num = round(norm(step_vec1)/interval)+1;
%         numvp = 2*cycle_num;
        step_size = norm(step_vec1)/(cycle_num-1);
        step_vec1 = step_vec1/norm(step_vec1);
        step_vec2 = step_vec2/norm(step_vec2);
        for idx=1:cycle_num
            pos1 = start_pos1+(idx-1)*step_size*step_vec1;
            pos2 = start_pos2+(idx-1)*step_size*step_vec2;
            z0 = pos1-origin;
            v1 = -z0;
            v2 = 0.5*(vision_pos(:,1)+vision_pos(:,2))-pos1;
            x0 = cross(v1,v2);
            y0 = cross(z0,x0);
            rot_mat = [x0/norm(x0), y0/norm(y0), z0/norm(z0)];
            rpy = tr2rpy(rot_mat, 'xyz');
            via_posrpy(:,2*idx-1) = [pos1; rpy'];
            via_posrpy(:,2*idx) = [pos2; rpy'];
        end
    elseif strcmp(type, 'washbowl')

    end

end