%   A specific planning method to clean runway mirror
%   seperate runway mirror to 3 zones: up, middle, down

function [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = PlanRunwayMirrorPath(vision_pos)
    % plan for middle zone
    path_type = 'n';
    camera_ori = 'top';
    dis_trans = 0.1;
    pitch_angle = deg2rad([30,50]);
    yaw_angle = deg2rad([0,0]);
    clean_tool = [0.25,0];
    trans_angle = deg2rad(30);
    RectZonePath = QuadranglePlanner;
    via_posrpy_middle = RectZonePath.UniversalPlan(vision_pos,clean_tool,pitch_angle,yaw_angle,...
                                    dis_trans,camera_ori,path_type,trans_angle);
    rot_runway = CalcRotRunway(vision_pos);
    % plan for up zone
    r = 0.5*norm(vision_pos(:,3)-vision_pos(:,4));
    origin = 0.5*(vision_pos(:,3)+vision_pos(:,4));
    arc_up = origin+r*rot_runway(:,1);
    arc_up = arc_up-0.5*clean_tool(1)*rot_runway(:,1);
    via_posrpy_up(1:3,2:4) = [vision_pos(:,3)-0.5*clean_tool(1)*rot_runway(:,1),...
                                arc_up,vision_pos(:,4)-0.5*clean_tool(1)*rot_runway(:,1)];
    via_posrpy_up(1:3,1) = vision_pos(:,3)+dis_trans*rot_runway(:,3);
    via_posrpy_up(1:3,5) = vision_pos(:,4)+dis_trans*rot_runway(:,3);
    rpy_arc = tr2rpy(rot_runway*[-1,0,0;0,1,0;0,0,-1]*rotx(-50), 'xyz');
    via_posrpy_up(4:6,:) = [rpy_arc',rpy_arc',rpy_arc',rpy_arc',rpy_arc'];
    % plan for down zone
    r = 0.5*norm(vision_pos(:,1)-vision_pos(:,2));
    origin = 0.5*(vision_pos(:,1)+vision_pos(:,2));
    arc_down = origin-r*rot_runway(:,1);
    arc_down = arc_down+0.5*clean_tool(1)*rot_runway(:,1);
    via_posrpy_down(1:3,2:4) = [vision_pos(:,2)+0.5*clean_tool(1)*rot_runway(:,1),...
                                arc_down,vision_pos(:,1)+0.5*clean_tool(1)*rot_runway(:,1)];
    via_posrpy_down(1:3,1) = vision_pos(:,2)+dis_trans*rot_runway(:,3);
    via_posrpy_down(1:3,5) = vision_pos(:,1)+dis_trans*rot_runway(:,3);
    rpy_arc = tr2rpy(rot_runway*[-1,0,0;0,1,0;0,0,-1]*rotx(-50), 'xyz');
    via_posrpy_down(4:6,:) = [rpy_arc',rpy_arc',rpy_arc',rpy_arc',rpy_arc'];
end

function rot_runway = CalcRotRunway(vertices)
    x0_runway = vertices(:,4)-vertices(:,1);
    z0_runway = cross(vertices(:,4)-vertices(:,2),vertices(:,3)-vertices(:,1));
    y0_runway = cross(z0_runway,x0_runway);
    rot_runway = [x0_runway/norm(x0_runway),y0_runway/norm(y0_runway),z0_runway/norm(z0_runway)];
end
