%   A specific planning method to clean ellipse mirror


function via_posrpy = PlanEllipseMirrorPath(vision_pos)

    truncation_width = 0.3;
    rot_ellipse = CalcRotEllipse(vision_pos);
    [a,b] = CalcEllipseParams(vision_pos);
    tmp_x = 0.5*truncation_width;
    tmp_y = sqrt(a^2*(1-tmp_x^2/b^2));
    vertices_rect = [rot_ellipse*[tmp_x;-tmp_y;0],rot_ellipse*[-tmp_x;-tmp_y;0],...
                        rot_ellipse*[-tmp_x;tmp_y;0],rot_ellipse*[tmp_x;tmp_y;0]];
    via_posrpy_rect = PlanRectZonePath(vertices_rect);
    tmp_rpy = via_posrpy_rect(4:6,2);
    via_posrpy_right = [via_posrpy_rect(:,2),[vision_pos(:,1);tmp_rpy],via_posrpy_rect(:,3)];
    via_posrpy_left = [via_posrpy_rect(:,end-2),[vision_pos(:,3);tmp_rpy],via_posrpy_rect(:,end-1)];

    via_posrpy = [via_posrpy_right,via_posrpy_rect,via_posrpy_left];
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

function [a,b] = CalcEllipseParams(vertices)
    origin = 0.5*(0.5*(vertices(:,1)+vertices(:,3))+0.5*(vertices(:,2)+vertices(:,4)));
    a = 0.5*(norm(origin-vertices(:,2))+norm(origin-vertices(:,4)));
    b = 0.5*(nomr(origin-vertices(:,1))+norm(origin-vertices(:,3)));
end