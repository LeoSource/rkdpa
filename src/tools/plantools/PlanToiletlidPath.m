% A specific planning method to open or close toilet lid
% arc_angle: positive value for open, negative for close
% point_angle: negative for open
% example argumets:
% open -> PlanToiletlidPath(vision_pos,110degree,-30degree,-40degree,0.03)
% close -> PlanToiletlidPath(vision_pos,-60degree,0,-40degree,0.03)

function via_posrpy = PlanToiletlidPath(vision_pos,arc_angle,point_angle,slant_angle,dis_trans)

% the 3rd point is the mark point: posC
% rotation aixs is the 2nd point to 1st point: posB->posA
posA = vision_pos(:,1); posB = vision_pos(:,2); posC = vision_pos(:,3);
% calculate the arc orientation and 3 points in arc
z0_axis = (posA-posB)/norm(posA-posB);
radius = norm(cross(z0_axis,posC-posB));
scale_line = dot(posC,z0_axis)-dot(posB,z0_axis);
posM = posB+scale_line*z0_axis;
x0_axis = (posC-posM)/norm(posC-posM);
y0_axis = cross(z0_axis,x0_axis);
rot_plane = [x0_axis,y0_axis,z0_axis];

via_posrpy(1:3,1) = posC-dis_trans*z0_axis;
via_posrpy(1:3,2) = posC;
tmp_pos(1,1) = radius*cos(arc_angle/2);
tmp_pos(2,1) = radius*sin(arc_angle/2);
tmp_pos(3,1) = 0;
tmp_pos = posM+rot_plane*tmp_pos;
via_posrpy(1:3,3) = tmp_pos+0.5*z0_axis*dis_trans;
tmp_pos(1) = radius*cos(arc_angle);
tmp_pos(2) = radius*sin(arc_angle);
tmp_pos(3) = 0;
tmp_pos = posM+rot_plane*tmp_pos;
via_posrpy(1:3,4) = tmp_pos+z0_axis*dis_trans;
% calculate orientation of the 3 points in arc
if arc_angle>0
    rot_rpy = rot_plane*rotz(90)*rotx(180/pi*point_angle)*roty(180/pi*slant_angle);
    rpy1 = tr2rpy(rot_rpy,'xyz');
    rpy_x0 = z0_axis;
    rpy_z0 = cross(z0_axis,tmp_pos-posM);
    rpy_z0 = rpy_z0/norm(rpy_z0);
    rpy_y0 = cross(rpy_z0,rpy_x0);
    rot_rpy = [rpy_x0,rpy_y0,rpy_z0];
    rpy2 = tr2rpy(rot_rpy,'xyz');
else
    rot_rpy = rot_plane*roty(180/pi*point_angle)*rotx(180/pi*slant_angle);
    rpy1 = tr2rpy(rot_rpy,'xyz');
    rpy_z0 = z0_axis;
    rpy_y0 = (posM-tmp_pos)/norm(posM-tmp_pos);
    rpy_x0 = cross(rpy_y0,rpy_z0);
    rot_rpy = [rpy_x0,rpy_y0,rpy_z0];
    rot_rpy = rot_rpy*roty(180/pi*point_angle)*rotx(180/pi*slant_angle);
    rpy2 = tr2rpy(rot_rpy, 'xyz');
end
via_posrpy(4:6,1) = rpy1;
via_posrpy(4:6,2) = rpy1;
via_posrpy(4:6,3) = rpy1;
via_posrpy(4:6,4) = rpy2;

end