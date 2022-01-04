%   A specific planning method to clean ellipse mirror
%   seperate ellipse mirror to 3 zones: up, middle, down


function [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = PlanEllipseMirrorPath(vision_pos)

    % plan for middle zone
    truncation_width = 0.4;
    rot_ellipse = CalcRotEllipse(vision_pos);
    [a,b,origin] = CalcEllipseParams(vision_pos);
    tmp_x = 0.5*truncation_width;
    tmp_y = sqrt(a^2*(1-tmp_x^2/b^2));
    vertices_rect = [origin+rot_ellipse*[tmp_x;-tmp_y;0],origin+rot_ellipse*[-tmp_x;-tmp_y;0],...
                        origin+rot_ellipse*[-tmp_x;tmp_y;0],origin+rot_ellipse*[tmp_x;tmp_y;0]];
    via_posrpy_rect = PlanRectZonePath(vertices_rect);
    tmp_rpy = via_posrpy_rect(4:6,2);
    clean_tool = [0.25,0];
    dis_trans = 0.1;
    via_pos_right = vision_pos(:,1)-rot_ellipse(:,1)*0.5*clean_tool(1);
    via_pos_left = vision_pos(:,3)+rot_ellipse(:,1)*0.5*clean_tool(1);
    via_posrpy_right = [via_posrpy_rect(:,1:2),[via_pos_right;tmp_rpy],via_posrpy_rect(:,3:4)];
    via_posrpy_left = [via_posrpy_rect(:,end-3:end-2),[via_pos_left;tmp_rpy],via_posrpy_rect(:,end-1:end)];
    via_posrpy_middle = [via_posrpy_right, via_posrpy_rect, via_posrpy_left];
    % plan for up zone
    r = 0.5*(norm(vertices_rect(:,3)-origin)+norm(vertices_rect(:,4)-origin));
    arc_up = origin+rot_ellipse(:,2)*r;
    vec_tmp = origin-vertices_rect(:,3);
    arc_left = vertices_rect(:,3)+0.5*clean_tool(1)*vec_tmp/norm(vec_tmp);
    vec_tmp = origin-arc_up;
    arc_up = arc_up+0.5*clean_tool(1)*vec_tmp/norm(vec_tmp);
    vec_tmp = origin-vertices_rect(:,4);
    arc_right = vertices_rect(:,4)+0.5*clean_tool(1)*vec_tmp/norm(vec_tmp);
    via_posrpy_up(1:3,2:4) = [arc_left, arc_up, arc_right];
    via_posrpy_up(1:3,1) = vertices_rect(:,3)+dis_trans*rot_ellipse(:,3);
    via_posrpy_up(1:3,5) = vertices_rect(:,4)+dis_trans*rot_ellipse(:,3);
    rpy_arc = tr2rpy(rot_ellipse*[0,-1,0;-1,0,0;0,0,-1]*rotx(-50),'xyz');
    via_posrpy_up(4:6,:) = [rpy_arc',rpy_arc',rpy_arc',rpy_arc',rpy_arc'];
    % plan for down zone
    r = 0.5*(norm(vertices_rect(:,1)-origin)+norm(vertices_rect(:,2)-origin));
    arc_down = origin-rot_ellipse(:,2)*r;
    vec_tmp = origin-vertices_rect(:,2);
    arc_left = vertices_rect(:,2)+0.5*clean_tool(1)*vec_tmp/norm(vec_tmp);
    vec_tmp = origin-arc_down;
    arc_down = arc_down+0.5*clean_tool(1)*vec_tmp/norm(vec_tmp);
    vec_tmp = origin-vertices_rect(:,1);
    arc_right = vertices_rect(:,1)+0.5*clean_tool(1)*vec_tmp/norm(vec_tmp);
    via_posrpy_down(1:3,2:4) = [arc_left, arc_down, arc_right];
    via_posrpy_down(1:3,1) = vertices_rect(:,2)+dis_trans*rot_ellipse(:,3);
    via_posrpy_down(1:3,5) = vertices_rect(:,1)+dis_trans*rot_ellipse(:,3);
    rpy_arc = tr2rpy(rot_ellipse*[0,-1,0;-1,0,0;0,0,-1]*rotx(-50),'xyz');
    via_posrpy_down(4:6,:) = [rpy_arc',rpy_arc',rpy_arc',rpy_arc',rpy_arc'];
end

function via_posrpy = PlanRectZonePath(vertices)
    path_type = 'n';
    camera_ori = 'top';
    dis_trans = 0.1;
    pitch_angle = deg2rad([30,50]);
    yaw_angle = [0,0];
    clean_tool = [0.25,0];
    trans_angle = deg2rad(30);
    RectZonePath = QuadranglePlanner;
    via_posrpy = RectZonePath.UniversalPlan(vertices,clean_tool,pitch_angle,yaw_angle,...
                                            dis_trans,camera_ori,path_type,trans_angle);
end

function rot_ellipse = CalcRotEllipse(vertices)
    origin = 0.5*(0.5*(vertices(:,1)+vertices(:,3))+0.5*(vertices(:,2)+vertices(:,4)));
    x0_ellipse = vertices(:,1)-origin;
    z0_ellipse = cross(x0_ellipse,vertices(:,4)-vertices(:,2));
    y0_ellipse = cross(z0_ellipse,x0_ellipse);
    rot_ellipse = [x0_ellipse/norm(x0_ellipse),y0_ellipse/norm(y0_ellipse),z0_ellipse/norm(z0_ellipse)];
end

function [a,b,origin] = CalcEllipseParams(vertices)
    origin = 0.5*(0.5*(vertices(:,1)+vertices(:,3))+0.5*(vertices(:,2)+vertices(:,4)));
    a = 0.5*(norm(origin-vertices(:,2))+norm(origin-vertices(:,4)));
    b = 0.5*(norm(origin-vertices(:,1))+norm(origin-vertices(:,3)));
end

