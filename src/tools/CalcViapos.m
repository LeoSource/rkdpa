function via_pos = CalcViapos(vision_pos, type)

    npos = size(vision_pos,2);
    if strcmp(type, 'toilet')
        center = mean(vision_pos, 2);
        rot_center = [0, 1, 0; 1, 0, 0; 0, 0, -1];
        rpy = tr2rpy(rot_center, 'xyz');
        via_pos(:,1) = [center; rpy'];
        theta = 20*pi/180;
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
            via_pos(:,idx+1) = [pos_tmp; rpy'];
        end
    end

end