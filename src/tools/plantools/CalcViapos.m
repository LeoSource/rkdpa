function via_posrpy = CalcViapos(vision_pos, type)

    npos = size(vision_pos,2);
    if strcmp(type, 'toilet')
        center = mean(vision_pos, 2);
        rot_center = [0, 1, 0; 1, 0, 0; 0, 0, -1];
        rpy1 = tr2rpy(rot_center, 'xyz');
        via_posrpy(:,1) = [center; rpy1'];
        arc_theta = 10*pi/180;%angle of vertical plane
        for idx=1:npos
            pos_tmp = vision_pos(:,idx);
            len = norm(center-pos_tmp);
            pos_high = center;
            pos_high(3) = center(3)+len*cot(arc_theta);
            z0 = pos_tmp-pos_high;
            z0 = z0/norm(z0);
%             y0 = CalcPlaneIntersection(z0,pos_high);
            y0 = cross(z0,[0,1,0]');
            y0 = y0/norm(y0);
            x0 = cross(y0,z0);
%             vec_high_center = center-pos_high;
%             x0 = cross(vec_high_center, z0);  
%             x0 = x0/norm(x0);
%             y0 = cross(z0,x0);
            rot_mat = [x0, y0, z0];
            rpy1 = tr2rpy(rot_mat, 'xyz');
            via_posrpy(:,idx+1) = [pos_tmp; rpy1'];
        end
    end

end

function vec = CalcPlaneIntersection(norm_vec, via_point)
    d = -dot(norm_vec,via_point);
    y = 0;
    x = via_point(1)+1;
    z = -(norm_vec(1)*x+d)/norm_vec(3);
    vec = [x;y;z]-via_point;
end

