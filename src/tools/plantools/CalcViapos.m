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
            y0 = CalcPlaneIntersection(z0,pos_high);
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
    elseif strcmp(type, 'toilet_lid')
        %the 3rd point is the mark point: posC
        %rotation axis is the 2nd point to 1st: posB->posA
        posA = vision_pos(:,1); posB = vision_pos(:,2); posC = vision_pos(:,3);
        arc_theta = vision_pos(1,end);
        slant = vision_pos(2,end);% positive for open, negative for close
        dis_trans = vision_pos(3,end);
        %calculate the arc orientation and 3 points in arc
        z0 = posA-posB;
        z0 = z0/norm(z0);
        radius = norm(cross(z0,posC-posB));
        scale = dot(posC,z0)-dot(posB,z0);
        posM = posB+scale*z0;
        x0 = posC-posM;
        x0 = x0/norm(x0);
        y0 = cross(z0,x0);
        rot_mat = [x0,y0,z0];
        via_posrpy(1:3,2) = posC;
        tmp_pos(1,1) = radius*cos(arc_theta/2);
        tmp_pos(2,1) = radius*sin(arc_theta/2);
        tmp_pos(3,1) = 0;
        tmp_pos = posM+rot_mat*tmp_pos;
%         dir_point = (posA-tmp_pos)/norm(posA-tmp_pos);
        via_posrpy(1:3,3) = tmp_pos+0.5*z0*dis_trans;
        tmp_pos(1,1) = radius*cos(arc_theta);
        tmp_pos(2,1) = radius*sin(arc_theta);
        tmp_pos(3,1) = 0;
        tmp_pos = posM+rot_mat*tmp_pos;
%         dir_point = (posA-tmp_pos)/norm(posA-tmp_pos);
        via_posrpy(1:3,4) = tmp_pos+z0*dis_trans;
        %calculae orientation of the 3 points in arc
        rpy_z0 = (posA-posC)/norm(posA-posC);
        via_posrpy(1:3,1) = posC-dis_trans*rpy_z0;
        rpy_y0 = cross(rpy_z0,z0);
        rpy_x0 = cross(rpy_y0,rpy_z0);
        tmp_mat = [rpy_x0, rpy_y0, rpy_z0];
        rpy_mat1 = tmp_mat*rotx(180/pi*slant);
        rpy1 = tr2rpy(rpy_mat1, 'xyz');
        rpy2 = tr2rpy(tmp_mat, 'xyz');
        via_posrpy(4:6,1) = rpy1;
        via_posrpy(4:6,2) = rpy1;
        via_posrpy(4:6,3) = rpy1;
        via_posrpy(4:6,4) = rpy2;
    end

end

function vec = CalcPlaneIntersection(norm_vec, via_point)
    d = -dot(norm_vec,via_point);
    y = 0;
    x = via_point(1)+1;
    z = -(norm_vec(1)*x+d)/norm_vec(3);
    vec = [x;y;z]-via_point;
end