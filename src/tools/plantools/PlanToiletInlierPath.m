function via_posrpy = PlanToiletInlierPath(vision_pos, slant, camera_ori)

    end_pos = vision_pos(:,end);
    np = size(vision_pos(:,1:end-1),2);
    pos_trans = mean(vision_pos(:,1:end-1),2);
    via_posrpy = zeros(6,3*np+2);
    via_posrpy(1:3,1) = pos_trans;
    via_posrpy(1:3,end) = pos_trans;
    for idx=1:np
        via_posrpy(1:3,3*idx-1) = vision_pos(:,idx);
        via_posrpy(1:3,3*idx) = end_pos;
        via_posrpy(1:3,3*idx+1) = vision_pos(:,idx);
    end
    if strcmp(camera_ori,'front')
        rot_transform = [0,1,0; 1,0,0; 0,0,-1];
        rot_mat = rot_transform*rotx(-slant*180/pi);
    elseif strcmp(camera_ori,'back')
        rot_transform = [0,-1,0; -1,0,0; 0,0,-1];
        rot_mat = rot_transform*rotx(slant*180/pi);
    elseif strcmp(camera_ori,'left')
        rot_transform = [-1,0,0; 0,1,0; 0,0,-1];
        rot_mat = rot_transform*roty(-slant*180/pi);
    elseif strcmp(camera_ori,'right')
        rot_transform = [1,0,0; 0,-1,0; 0,0,-1];
        rot_mat = rot_transform*roty(slant*180/pi);
    end
    rpy = tr2rpy(rot_mat, 'xyz');
    for idx=1:size(via_posrpy,2)
        via_posrpy(4:6,idx) = rpy;
    end
end