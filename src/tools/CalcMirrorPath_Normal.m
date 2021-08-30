function via_posrpy = CalcMirrorPath_Normal(corner_pos,lenscraper,inc_ang,dis_trans)
% the sequence of corner position is like this
% 3--------4
% |        |
% |        |
% 2--------1
% lenscraper: length of scraper
% inc_ang: included angle between mirror and tool axis
% dis_trans: transition distance in z-axis before scrape mirror
x0_mirror = corner_pos(:,4)-corner_pos(:,1);
x0_mirror = x0_mirror/norm(x0_mirror);
y0_mirror = corner_pos(:,2)-corner_pos(:,1);
y0_mirror = y0_mirror/norm(y0_mirror);
z0_mirror = cross(x0_mirror,y0_mirror);
rot_mirror = [x0_mirror,y0_mirror,z0_mirror];

corner_pos_new(:,1) = corner_pos(:,1)+lenscraper/2*y0_mirror;
corner_pos_new(:,2) = corner_pos(:,2)-lenscraper/2*y0_mirror;
corner_pos_new(:,3) = corner_pos(:,3)-lenscraper/2*y0_mirror;
corner_pos_new(:,4) = corner_pos(:,4)+lenscraper/2*y0_mirror;

width_target = norm(corner_pos_new(:,1)-corner_pos_new(:,2));
cycle_num = ceil(width_target/lenscraper)+1;
step_size = width_target/(cycle_num-1);
start_pos1 = corner_pos_new(:,4);
start_pos2 = corner_pos_new(:,1);
trans_vec = z0_mirror*dis_trans;
rot_transform = [0,0,1; 1,0,0; 0,1,0];
for idx=1:cycle_num
    pos2 = start_pos1+(idx-1)*step_size*y0_mirror;
    pos1 = pos2 + trans_vec;
    pos3 = start_pos2+(idx-1)*step_size*y0_mirror;
    pos4 = pos3+trans_vec;
    rot_tool = rot_mirror*roty(inc_ang*180/pi)*rot_transform;
    rpy = tr2rpy(rot_tool, 'xyz');
    via_posrpy(:,4*idx-3) = [pos1; rpy'];
    via_posrpy(:,4*idx-2) = [pos2; rpy'];
    via_posrpy(:,4*idx-1) = [pos3; rpy'];
    via_posrpy(:,4*idx) = [pos4; rpy'];
end


end