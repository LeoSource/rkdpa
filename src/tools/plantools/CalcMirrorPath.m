function via_posrpy = CalcMirrorPath(corner_pos,lenscraper,slant_ang,inc_ang)
% the sequence of corner position is like this
% 3--------4
% |        |
% |        |
% 2--------1
% lenscraper: length of scraper
% slant_ang: slant angle between scraper axis and mirror height vector
% inc_ang: included angle between mirror and tool axis
x0_mirror = corner_pos(:,4)-corner_pos(:,1);
x0_mirror = x0_mirror/norm(x0_mirror);
y0_mirror = corner_pos(:,2)-corner_pos(:,1);
y0_mirror = y0_mirror/norm(y0_mirror);
z0_mirror = cross(x0_mirror,y0_mirror);
rot_mirror = [x0_mirror,y0_mirror,z0_mirror];

corner_pos_new(:,1) = corner_pos(:,1)+lenscraper/2*x0_mirror+lenscraper/2*y0_mirror;
corner_pos_new(:,2) = corner_pos(:,2)+lenscraper/2*x0_mirror-lenscraper/2*y0_mirror;
corner_pos_new(:,3) = corner_pos(:,3)-lenscraper/2*x0_mirror-lenscraper/2*y0_mirror;
corner_pos_new(:,4) = corner_pos(:,4)-lenscraper/2*x0_mirror+lenscraper/2*y0_mirror;

height_target = norm(corner_pos_new(:,2)-corner_pos_new(:,3));
cycle_num = ceil(height_target/lenscraper)+1;
step_size = height_target/(cycle_num-1);
start_pos1 = corner_pos_new(:,3);
start_pos2 = corner_pos_new(:,4);
rot_transform = [1,0,0;0,-1,0;0,0,-1];
for idx=1:cycle_num
    if mod(idx,2)==1
        pos1 = start_pos1+(idx-1)*step_size*(-x0_mirror);
        pos2 = start_pos2+(idx-1)*step_size*(-x0_mirror);
        rot_tool = rot_mirror*rotz(180-slant_ang*180/pi)*rotx(-inc_ang*180/pi)*rot_transform;
    else
        pos1 = start_pos2+(idx-1)*step_size*(-x0_mirror);
        pos2 = start_pos1+(idx-1)*step_size*(-x0_mirror);
        rot_tool = rot_mirror*rotz(slant_ang*180/pi)*rotx(-inc_ang*180/pi)*rot_transform;
    end
    rpy = tr2rpy(rot_tool, 'xyz');
    via_posrpy(:,2*idx-1) = [pos1; rpy'];
    via_posrpy(:,2*idx) = [pos2; rpy'];
end



end