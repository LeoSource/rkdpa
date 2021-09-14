function via_posrpy = PlanUniversalRectPath(vertices, clean_tool, pitch_angle, yaw_angle, dis_trans, camera_ori)
% the sequence of vertices is like this
% 3--------4
% |        |
% |        |
% 2--------1
% clean_tool: the size of clean tool, includes legnth for direction y and width for direction x
% yaw_angle(rotz): when it is bigger than pi or less than -pi, it will be defined
% by an adaptive method, otherwise, use its input variable to define orientation
% pitch_angle(roty): it dose not hava an adaptive method to set pitch
% for example:
% mirror: pitch_angle = [60degree, 60degree] yaw_angle = [0,0]
% table: pitch_angle = [50degree, 130degree] yaw_angle = [-10,0]
% dis_trans: transition distance with surface constructed bt vertices
% camera_ori: includes 4 types, front, down, left, right
x0_plane = vertices(:,4)-vertices(:,1);
x0_plane = x0_plane/norm(x0_plane);
y0_plane = vertices(:,2)-vertices(:,1);
y0_plane = y0_plane/norm(y0_plane);
z0_plane = cross(x0_plane,y0_plane);
rot_plane = [x0_plane, y0_plane, z0_plane];
% simplify the clean area model
l = clean_tool(1); w = clean_tool(2);
vertices_new(:,1) = vertices(:,1)+w/2*x0_plane+l/2*y0_plane;
vertices_new(:,2) = vertices(:,2)+w/2*x0_plane-l/2*y0_plane;
vertices_new(:,3) = vertices(:,3)-w/2*x0_plane-l/2*y0_plane;
vertices_new(:,4) = vertices(:,4)-w/2*x0_plane+l/2*y0_plane;
% TO DO: add 's' pathtype
width_target = norm(vertices_new(:,2)-vertices_new(:,1));
cycle_num = ceil(width_target/l)+1;
step_size = width_target/(cycle_num-1);
start_pos1 = vertices_new(:,4);
start_pos2 = vertices_new(:,1);
switch camera_ori
    case 'top'
        rot_transform = [0,0,1; 1,0,0; 0,1,0];
    case 'left'
        rot_transform = [0,0,1; 0,1,0; -1,0,0];
    case 'down'
        rot_transform = [0,0,1; -1,0,0; 0,-1,0];
    case 'right'
        rot_transform = [0,0,1; 0,-1,0; 1,0,0];
end
if dis_trans>0
    trans_vec = z0_plane*dis_trans;
    for idx=1:cycle_num
        pos2 = start_pos1+(idx-1)*step_size*y0_plane;
        pos1 = pos2+trans_vec;
        pos3 = start_pos2+(idx-1)*step_size*y0_plane;
        pos4 = pos3+trans_vec;
        mid_len = 0.5*(vertices_new(:,1)+vertices_new(:,2));
        yaw = CalcRectYaw(mid_len, pos2, pos3, rot_plane, yaw_angle, cycle_num, idx);
%         if yaw_angle(1)>pi || yaw_angle(1)<-pi
%             mid_len = 0.5*(vertices_new(:,1)+vertices_new(:,2));
%             tmp_vec1 = mid_len-pos1;
%             if norm(tmp_vec1)<1e-6
%                 cos_theta = 1;
%             else
%                 cos_theta = dot(tmp_vec1,-x0_plane)/norm(tmp_vec1);
%             end
%             dir = cross(tmp_vec1,-x0_plane)./z0_plane;
%             yaw1 = sign(dir(3))*acos(cos_theta);
%         else
%             step_yaw1 = 2*yaw_angle(1)/(cycle_num-1);
%             yaw1 = yaw_angle(1)-(idx-1)*step_yaw1;
%         end
%         if yaw_angle(2)>pi || yaw_angle(2)<-pi
%             mid_len = 0.5*(vertices_new(:,1)+vertices_new(:,2));
%             tmp_vec2 = mid_len-pos2;
%             if norm(tmp_vec2)<1e-6
%                 cos_theta = 1;
%             else
%                 cos_theta = dot(tmp_vec2,-x0_plane)/norm(tmp_vec2);
%             end
%             dir = cross(tmp_vec2,-x0_plane)./z0_plane;
%             yaw2 = sign(dir(3))*acos(cos_theta);
%         else
%             step_yaw2 = 2*yaw_angle(2)/(cycle_num-1);
%             yaw2 = yaw_angle(2)-(idx-1)*step_yaw2;
%         end
        pitch1 = pitch_angle(1);
        pitch2 = pitch_angle(2);
        yaw1 = yaw(1);
        yaw2 = yaw(2);
        rot_tool1 = rot_plane*rotz(-180/pi*yaw1)*roty(180/pi*pitch1)*rot_transform;
        rot_tool2 = rot_plane*rotz(-180/pi*yaw2)*roty(180/pi*pitch2)*rot_transform;
        rpy1 = tr2rpy(rot_tool1, 'xyz');
        rpy2 = tr2rpy(rot_tool2, 'xyz');
        via_posrpy(:,4*idx-3) = [pos1; rpy1'];
        via_posrpy(:,4*idx-2) = [pos2; rpy1'];
        via_posrpy(:,4*idx-1) = [pos3; rpy2'];
        via_posrpy(:,4*idx) = [pos4; rpy2'];
    end
else
    for idx=1:cycle_num
        pos1 = start_pos1+(idx-1)*step_size*y0_plane;
        pos2 = start_pos2+(idx-1)*step_size*y0_plane;
        mid_len = 0.5*(vertices_new(:,1)+vertices_new(:,2));
        yaw = CalcRectYaw(mid_len, pos1, pos2, rot_plane, yaw_angle, cycle_num, idx);
%         if yaw_angle(1)>pi || yaw_angle(1)<-pi
%             mid_len = 0.5*(vertices_new(:,1)+vertices_new(:,2));
%             tmp_vec1 = mid_len-pos1;
%             if norm(tmp_vec1)<1e-6
%                 cos_theta = 1;
%             else
%                 cos_theta = dot(tmp_vec1,-x0_plane)/norm(tmp_vec1);
%             end
%             dir = cross(tmp_vec1,-x0_plane)./z0_plane;
%             yaw1 = sign(dir(3))*acos(cos_theta);
%         else
%             step_yaw1 = 2*yaw_angle(1)/(cycle_num-1);
%             yaw1 = yaw_angle(1)-(idx-1)*step_yaw1;
%         end
%         if yaw_angle(2)>pi || yaw_angle(2)<-pi
%             mid_len = 0.5*(vertices_new(:,1)+vertices_new(:,2));
%             tmp_vec2 = mid_len-pos2;
%             if norm(tmp_vec2)<1e-6
%                 cos_theta = 1;
%             else
%                 cos_theta = dot(tmp_vec2,-x0_plane)/norm(tmp_vec2);
%             end
%             dir = cross(tmp_vec2,-x0_plane)./z0_plane;
%             yaw2 = sign(dir(3))*acos(cos_theta);
%         else
%             step_yaw2 = 2*yaw_angle(2)/(cycle_num-1);
%             yaw2 = yaw_angle(2)-(idx-1)*step_yaw2;
%         end
        pitch1 = pitch_angle(1);
        pitch2 = pitch_angle(2);
        yaw1 = yaw(1);
        yaw2 = yaw(2);
        rot_tool1 = rot_plane*rotz(-180/pi*yaw1)*roty(180/pi*pitch1)*rot_transform;
        rot_tool2 = rot_plane*rotz(-180/pi*yaw2)*roty(180/pi*pitch2)*rot_transform;
        rpy1 = tr2rpy(rot_tool1, 'xyz');
        rpy2 = tr2rpy(rot_tool2, 'xyz');
        via_posrpy(:,2*idx-1) = [pos1; rpy1'];
        via_posrpy(:,2*idx) = [pos2; rpy2'];
    end
    
end

end

function yaw = CalcRectYaw(origin, pos1, pos2, rot_plane, yaw_angle, ncycle, index)
    x0_axis = rot_plane(:,1); z0_axis = rot_plane(:,3);
    if yaw_angle(1)>pi || yaw_angle(1)<-pi
%         origin = 0.5*(origin(:,1)+origin(:,2));
        tmp_vec1 = origin-pos1;
        if norm(tmp_vec1)<1e-6
            cos_theta = 1;
        else
            cos_theta = dot(tmp_vec1,-x0_axis)/norm(tmp_vec1);
        end
        dir = cross(tmp_vec1,-x0_axis)./z0_axis;
        yaw(1) = sign(dir(3))*acos(cos_theta);
    else
        step_yaw1 = 2*yaw_angle(1)/(ncycle-1);
        yaw(1) = yaw_angle(1)-(index-1)*step_yaw1;
    end
    if yaw_angle(2)>pi || yaw_angle(2)<-pi
%         origin = 0.5*(origin(:,1)+origin(:,2));
        tmp_vec2 = origin-pos2;
        if norm(tmp_vec2)<1e-6
            cos_theta = 1;
        else
            cos_theta = dot(tmp_vec2,-x0_axis)/norm(tmp_vec2);
        end
        dir = cross(tmp_vec2,-x0_axis)./z0_axis;
        yaw(2) = sign(dir(3))*acos(cos_theta);
    else
        step_yaw2 = 2*yaw_angle(2)/(ncycle-1);
        yaw(2) = yaw_angle(2)-(index-1)*step_yaw2;
    end
end

function yaw = CalcAdaptiveYaw(origin, pos, rot_plane)
    x0_axis = rot_plane(:,1); z0_axis = rot_plane(:,3);
    tmp_vec = origin-pos;
    if norm(tmp_vec)<1e-6
        cos_theta = 1;
    else
        cos_theta = dot(tmp_vec,-x0_axis)/norm(tmp_vec);
    end
    dir = cross(tmp_vec,-x0_axis)./z0_axis;
    yaw = sign(dir(3))*acos(cos_theta);
end
