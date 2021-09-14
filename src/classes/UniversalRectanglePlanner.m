%   A specific class that generate positions and orientations to guarantee that all the positions
%   can cover a given rectangle area
%   the sequence of rectangle vertices is like this
%   3！！！！！！！4
%   |       x^
%   |        |
%   | y      |
%   2<！！！！！！1
%   clean_tool: the size of clean tool, includes legnth for direction y and width for direction x
%   yaw_angle(rotz): when it is bigger than pi or less than -pi, it will be defined
%   by an adaptive method, otherwise, use its input variable to define orientation
%   pitch_angle(roty): it dose not hava an adaptive method to set pitch
%   for example:
%   mirror: pitch_angle = [60degree, 60degree] yaw_angle = [0,0]
%   table: pitch_angle = [50degree, 130degree] yaw_angle = [-10,0]
%   dis_trans: transition distance with surface constructed bt vertices
%   camera_ori: includes 4 types, front, down, left, right
%   Author:
%   Liao Zhixiang, zhixiangleo@163.com

classdef UniversalRectanglePlanner < handle
    properties
        rot_plane
        vertices_new
    end

    methods
        function obj = UniversalRectanglePlanner()

        end

        function via_posrpy = UniversalPlan(vertices,clean_tool,pitch_angle,...
                                            yaw_angle,dis_trans,camera_ori,path_type)

            x0_plane = vertices(:,4)-vertices(:,1);
            y0_plane = vertices(:,2)-vertices(:,1);
            z0_plane = cross(x0_plane,y0_plane);
            obj.rot_plane = [x0_plane/norm(x0_plane),y0_plane/norm(y0_plane),z0_plane/norm(z0_plane)];
            % simplify the clean area model
            l = clean_tool; w = clean_tool(2);
            obj.vertices_new(:,1) = vertices(:,1)+w/2*x0_plane+l/2*y0_plane;
            obj.vertices_new(:,2) = vertices(:,2)+w/2*x0_plane-l/2*y0_plane;
            obj.vertices_new(:,3) = vertices(:,3)-w/2*x0_plane-l/2*y0_plane;
            obj.vertices_new(:,4) = vertices(:,4)-w/2*x0_plane+l/2*y0_plane;

            if path_type=='n'
                width_target = norm(obj.vertices_new(:,2)-obj.vertices_new(:,1));
                cycle_num = ceil(width_target/l)+1;
                step_size = width_target/(cycle_num-1);
                step_vec = y0_plane;
                start_pos1 = obj.vertices_new(:,4);
                start_pos2 = obj.vertices_new(:,1);
            elseif path_type=='s'
                width_target = norm(obj.vertices_new(:,2)-obj.vertices_new(:,3));
                cycle_num = ceil(width_target/w)+1;
                step_size = width_target/(cycle_num-1);
                step_vec = -x0_plane;
                start_pos1 = obj.vertices_new(:,4);
                start_pos2 = obj.vertices_new(:,3);
            end
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
                    pos2 = start_pos1+(idx-1)*step_size*step_vec;
                    pos1 = pos2+trans_vec;
                    pos3 = start_pos2+(idx-1)*step_size*step_vec;
                    pos4 = pos3+trans_vec;
                    if path_type=='n'
                        pitch1 = pitch_angle(1);
                        pitch2 = pitch_angle(2);
                        yaw1 = CalcNPathYaw(pos2, yaw_angle(1), cycle_num, idx);
                        yaw2 = CalcNPathYaw(pos3, yaw_angle(2), cycle_num, idx);
                    elseif path_type=='s'
                        pitch1 = CalcSPathPitch(pitch_angle, cycle_num, idx);
                        pitch2 = pitch1;
                        yaw1 = CalcSPathYaw(pos2, yaw_angle, cycle_num, idx);
                        yaw2 = -yaw1;
                    end
                    rot_tool1 = obj.rot_plane*rotz(-180/pi*yaw1)*roty(180/pi*pitch1)*rot_transform;
                    rot_tool2 = obj.rot_plane*rotz(-180/pi*yaw2)*roty(180/pi*pitch2)*rot_transform;
                    rpy1 = tr2rpy(rot_tool1, 'xyz');
                    rpy2 = tr2rpy(rot_tool2, 'xyz');
                    via_posrpy(:,4*idx-3) = [pos1; rpy1'];
                    via_posrpy(:,4*idx-2) = [pos2; rpy1'];
                    via_posrpy(:,4*idx-1) = [pos3; rpy2'];
                    via_posrpy(:,4*idx) = [pos4; rpy2'];
                end
            else
                for idx=1:cycle_num
                    pos1 = start_pos1+(idx-1)*step_size*step_vec;
                    pos2 = start_pos2+(idx-1)*step_size*step_vec;
                    if path_type=='n'
                        pitch1 = pitch_angle(1);
                        pitch2 = pitch_angle(2);
                        yaw1 = CalcNPathYaw(pos1, yaw_angle(1), cycle_num, idx);
                        yaw2 = CalcNPathYaw(pos2, yaw_angle(2), cycle_num, idx);
                    elseif path_type=='s'
                        pitch1 = CalcSPathPitch(pitch_angle, cycle_num, idx);
                        pitch2 = pitch1;
                        yaw1 = CalcSPathYaw(pos1, yaw_angle, cycle_num, idx);
                        yaw2 = -yaw1;
                    end
                    rot_tool1 = obj.rot_plane*rotz(-180/pi*yaw1)*roty(180/pi*pitch1)*rot_transform;
                    rot_tool2 = obj.rot_plane*rotz(-180/pi*yaw2)*roty(180/pi*pitch2)*rot_transform;
                    rpy1 = tr2rpy(rot_tool1, 'xyz');
                    rpy2 = tr2rpy(rot_tool2, 'xyz');
                    via_posrpy(:,2*idx-1) = [pos1; rpy1'];
                    via_posrpy(:,2*idx) = [pos2; rpy2'];
                end
                
            end

        end

        function yaw = CalcNPathYaw(obj, pos, yaw_angle, ncycle, index)
            if yaw_angle>pi || yaw_angle<-pi
                origin = 0.5*(obj.vertices_new(:,1)+obj.vertices_new(:,2));
                yaw = CalcAdaptiveYaw(origin, pos);
            else
                step_yaw = 2*yaw_angle/(ncycle-1);
                yaw = yaw_angle-(index-1)*step_yaw;
            end
        end

        function yaw = CalcSPathYaw(obj, pos, yaw_range, ncycle, index)
            if yaw_range(1)>pi || yaw_range(1)<-pi
                origin = 0.5*(obj.vertices_new(:,1)+obj.vertices_new(:,2));
                yaw = CalcAdaptiveYaw(origin, pos);
            else
                step_yaw = abs(yaw_range(2)-yaw_range(1))/(ncycle-1);
                yaw = yaw_range(1)-(index-1)*step_yaw;
            end
        end

        function yaw = CalcAdaptiveYaw(obj, origin, pos)
            x0_axis = obj.rot_plane(:,1); z0_axis = obj.rot_plane(:,3);
            tmp_vec = origin-pos;
            if norm(tmp_vec)<1e-6
                cos_theta = 1;
            else
                cos_theta = dot(tmp_vec,-x0_axis)/norm(tmp_vec);
            end
            dir = cross(tmp_vec,-x0_axis)./z0_axis;
            yaw = sign(dir(3))*acos(cos_theta);
        end

        function pitch = CalcSPathPitch(pitch_range, ncycle, index)
            step_pitch = (pitch_range(2)-pitch_range(1))/(ncycle-1);
            pitch = pitch_range(1)+(index-1)*step_pitch;
        end

        function via_posrpy = PlanMirror()


        end

        function via_posrpy = PlanTable()

        end

        function via_posrpy = PlanGround()

        end

    end

end
