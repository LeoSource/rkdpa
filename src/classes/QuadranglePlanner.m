%   A specific class that generate positions and orientations to guarantee that all the positions
%   can cover a given quadrangle area
%   the sequence of quadrangle vertices is like this
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

classdef QuadranglePlanner < handle
    properties
        rot_plane
        rot_transform
        vertices_new

        cycle_num
        step_size1
        step_size2
        step_vec1
        step_vec2
        start_pos1
        start_pos2
    end

    methods
        function obj = QuadranglePlanner()

        end

        function via_posrpy = UniversalPlan(obj, vertices,clean_tool,pitch_angle,...
                                            yaw_angle,dis_trans,camera_ori,path_type)
            obj.CalcPlaneRot(vertices);
            obj.CalcCleanAreaVertices(vertices,clean_tool);
            obj.SetRotTransformation(camera_ori);
            if path_type=='n'
                via_posrpy = obj.PlanNPath(clean_tool, pitch_angle, yaw_angle, dis_trans);
            elseif path_type=='s'
                via_posrpy = obj.PlanSPath(clean_tool, pitch_angle, yaw_angle, dis_trans);
            end

        end

        function via_posrpy = PlanNPath(obj, clean_tool, pitch_angle, yaw_angle, dis_trans)
            obj.CalcCycleInfo(clean_tool(1), obj.vertices_new(:,3)-obj.vertices_new(:,4), ...
                                            obj.vertices_new(:,2)-obj.vertices_new(:,1));
            obj.start_pos1 = obj.vertices_new(:,4);
            obj.start_pos2 = obj.vertices_new(:,1);
            if dis_trans>0
                trans_vec = dis_trans*obj.rot_plane(:,3);
                for idx=1:obj.cycle_num
                    waypoint = obj.CalcDisCycleWaypoint(pitch_angle,yaw_angle,trans_vec,'n');
                    via_posrpy(:,4*idx-3:4*idx) = waypoint;
                end
            else
                for idx=1:obj.cycle_num
                    waypoint = obj.CalcCycleWaypoint(pitch_angle, yaw_angle, idx, 'n');
                    via_posrpy(:,2*idx-1:2*idx) = waypoint;
                end
            end
        end

        function via_posrpy = PlanSPath(obj, clean_tool, pitch_angle, yaw_angle, dis_trans)
            obj.CalcCycleInfo(clean_tool(2), obj.vertices_new(:,1)-obj.vertices_new(:,4), ...
                                            obj.vertices_new(:,3)-obj.vertices_new(:,2));
            obj.start_pos1 = obj.vertices_new(:,4);
            obj.start_pos2 = obj.vertices_new(:,3);
            if dis_trans>0
                trans_vec = dis_trans*obj.rot_plane(:,3);
                for idx=1:obj.cycle_num
                    waypoint = obj.CalcDisCycleWaypoint(pitch_angle,yaw_angle,trans_vec,'s');
                    via_posrpy(:,4*idx-3:4*idx) = waypoint;
                end
            else
                for idx=1:obj.cycle_num
                    waypoint = obj.CalcCycleWaypoint(pitch_angle,yaw_angle,idx,'s');
                    via_posrpy(:,2*idx-1:2*idx) = waypoint;
                end
            end
        end

        function waypoint = CalcCycleWaypoint(obj, pitch_angle, yaw_angle, idx, path_type)
            pos1 = obj.start_pos1+(idx-1)*obj.step_size1*obj.step_vec1;
            pos2 = obj.start_pos2+(idx-1)*obj.step_size2*obj.step_vec2;
            if path_type=='n'
                [pitch1,pitch2,yaw1,yaw2] = obj.CalcNPathRot(pitch_angle,yaw_angle,pos1,pos2,idx);
            elseif path_type=='s'
                [pitch1,pitch2,yaw1,yaw2] = obj.CalcSPathRot(pitch_angle, yaw_angle, pos1, idx);
            end
            rot_tool1 = obj.rot_plane*rotz(-180/pi*yaw1)*roty(180/pi*pitch1)*obj.rot_transform;
            rot_tool2 = obj.rot_plane*rotz(-180/pi*yaw2)*roty(180/pi*pitch2)*obj.rot_transform;
            rpy1 = tr2rpy(rot_tool1, 'xyz');
            rpy2 = tr2rpy(rot_tool2, 'xyz');
            waypoint(:,1) = [pos1; rpy1'];
            waypoint(:,2) = [pos2; rpy2'];
        end

        function waypoint = CalcDisCycleWaypoint(obj, pitch_angle, yaw_angle, trans_vec, path_type)
            pos2 = obj.start_pos1+(idx-1)*obj.step_size1*obj.step_vec1;
            pos1 = pos2+trans_vec;
            pos3 = obj.start_pos2+(idx-1)*obj.step_size2*obj.step_vec2;
            pos4 = pos3+trans_vec;
            if path_type=='n'
                [pitch1,pitch2,yaw1,yaw2] = obj.CalcNPathRot(pitch_angle,yaw_angle,pos2,pos3,idx);
            elseif path_type=='s'
                [pitch1,pitch2,yaw1,yaw2] = obj.CalcSPathRot(pitch_angle, yaw_angle, pos2, idx);
            end
            rot_tool1 = obj.rot_plane*rotz(-180/pi*yaw1)*roty(180/pi*pitch1)*obj.rot_transform;
            rot_tool2 = obj.rot_plane*rotz(-180/pi*yaw2)*roty(180/pi*pitch2)*obj.rot_transform;
            rpy1 = tr2rpy(rot_tool1, 'xyz');
            rpy2 = tr2rpy(rot_tool2, 'xyz');
            via_posrpy(:,4*idx-3) = [pos1; rpy1'];
            via_posrpy(:,4*idx-2) = [pos2; rpy1'];
            via_posrpy(:,4*idx-1) = [pos3; rpy2'];
            via_posrpy(:,4*idx) = [pos4; rpy2'];
        end

        function CalcCycleInfo(obj, interval, vec1, vec2)
            len1 = norm(vec1);
            len2 = norm(vec2);
            cycle_num1 = ceil(len1/interval)+1;
            cycle_num2 = ceil(len2/interval)+1;
            obj.cycle_num = max(cycle_num1,cycle_num2);
            obj.step_size1 = len1/(obj.cycle_num-1);
            obj.step_size2 = len2/(obj.cycle_num-1);
            obj.step_vec1 = vec1/len1;
            obj.step_vec2 = vec2/len2;
        end

        function [pitch1, pitch2, yaw1, yaw2] = CalcNPathRot(obj, pitch_angle, yaw_angle, pos1, pos2, idx)
            pitch1 = pitch_angle(1);
            pitch2 = pitch_angle(2);
            yaw1 = obj.CalcNPathYaw(pos1, yaw_angle(1), idx);
            yaw2 = obj.CalcNPathYaw(pos2, yaw_angle(2), idx);
        end

        function [pitch1, pitch2, yaw1, yaw2] = CalcSPathRot(obj, pitch_angle, yaw_angle, pos, idx)
            pitch1 = obj.CalcSPathPitch(pitch_angle, idx);
            pitch2 = pitch1;
            yaw1 = obj.CalcSPathYaw(pos, yaw_angle, idx);
            yaw2 = -yaw1;
        end

        function CalcPlaneRot(obj, vertices)
            x0_plane = vertices(:,4)-vertices(:,1);
            z0_plane = cross(x0_plane, vertices(:,2)-vertices(:,1));
            y0_plane = cross(z0_plane, x0_plane);
            obj.rot_plane = [x0_plane/norm(x0_plane), y0_plane/norm(y0_plane), z0_plane/norm(z0_plane)];
        end

        function yaw = CalcNPathYaw(obj, pos, yaw_angle, index)
            if yaw_angle>pi || yaw_angle<-pi
                origin = 0.5*(obj.vertices_new(:,1)+obj.vertices_new(:,2));
                yaw = CalcAdaptiveYaw(origin, pos);
            else
                step_yaw = 2*yaw_angle/(obj.cycle_num-1);
                yaw = yaw_angle-(index-1)*step_yaw;
            end
        end

        function yaw = CalcSPathYaw(obj, pos, yaw_range, index)
            if yaw_range(1)>pi || yaw_range(1)<-pi
                origin = 0.5*(obj.vertices_new(:,1)+obj.vertices_new(:,2));
                yaw = CalcAdaptiveYaw(origin, pos);
            else
                step_yaw = abs(yaw_range(2)-yaw_range(1))/(obj.cycle_num-1);
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

        function pitch = CalcSPathPitch(obj, pitch_range, ncycle, index)
            step_pitch = (pitch_range(2)-pitch_range(1))/(ncycle-1);
            pitch = pitch_range(1)+(index-1)*step_pitch;
        end

        function CalcCleanAreaVertices(obj, vertices, clean_tool)
            % simplify the clean area model
            l = clean_tool(1); w = clean_tool(2);
            x0_plane = obj.rot_plane(:,1);
            y0_plane = obj.rot_plane(:,2);
            obj.vertices_new(:,1) = vertices(:,1)+w/2*x0_plane+l/2*y0_plane;
            obj.vertices_new(:,2) = vertices(:,2)+w/2*x0_plane-l/2*y0_plane;
            obj.vertices_new(:,3) = vertices(:,3)-w/2*x0_plane-l/2*y0_plane;
            obj.vertices_new(:,4) = vertices(:,4)-w/2*x0_plane+l/2*y0_plane;
        end

        function SetRotTransformation(obj, camera_ori)
            switch camera_ori
            case 'top'
                obj.rot_transform = [0,0,1; 1,0,0; 0,1,0];
            case 'left'
                obj.rot_transform = [0,0,1; 0,1,0; -1,0,0];
            case 'down'
                obj.rot_transform = [0,0,1; -1,0,0; 0,-1,0];
            case 'right'
                obj.rot_transform = [0,0,1; 0,-1,0; 1,0,0];
            end
        end

        function via_posrpy = PlanMirror(obj, vertices)
            path_type = 'n';
            camera_ori = 'top';
            dis_trans = 0.08;
            pitch_angle = [60*pi/180, 60*pi/180];
            yaw_angle = [0, 0];
            clean_tool = [0.15, 0];
            via_posrpy = obj.UniversalPlan(vertices,clean_tool,pitch_angle,...
                                        yaw_angle,dis_trans,camera_ori,path_type);
        end

        function via_posrpy = PlanTable(obj, vertices)
            path_type = 'n';
            camera_ori = 'top';
            dis_trans = -1;
            pitch_angle = [50*pi/180, pi-50*pi/180];
            yaw_angle = [70*pi/180, 10*pi/180];
            clean_tool = [0.2, 0.15];
            via_posrpy = obj.UniversalPlan(vertices,clean_tool,pitch_angle,...
                                        yaw_angle,dis_trans,camera_ori,path_type);
        end

        function via_posrpy = PlanGround(obj, vertices)
            path_type = 'n';
            camera_ori = 'top';
            dis_trans = -1;
            pitch_angle = [50*pi/180, 80*pi/180];
            yaw_angle = [70*pi/180, 10*pi/180];
            clean_tool = [0.2, 0.15];
            via_posrpy = obj.UniversalPlan(vertices,clean_tool,pitch_angle,...
                                        yaw_angle,dis_trans,camera_ori,path_type);
        end

    end

end
