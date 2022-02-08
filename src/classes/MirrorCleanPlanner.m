%   A specific class that will plan clean path for several kinds of mirrors
%   It contains 5 kinds of regular mirrors: rectangle, circle, ellipse,
%   runway, octagon
%   This method separate mirror clean area to rectangle and other
%   corresponding area
%   Author:
%   Liao Zhixiang, zhixiangleo@163.com

classdef MirrorCleanPlanner < handle
    properties
        vertices
        type
        clean_params
        hori_times

        rot_plane
        ellipse_params
        circle_params
    end
    
    
    methods
        function obj = MirrorCleanPlanner()
            
        end
    
        function SetCleanParams(obj,vertices,type,clean_params)
            obj.vertices = vertices;
            obj.type = type;
            obj.hori_times = 1;
            if nargin==4
                obj.clean_params = clean_params;
            else
                if strcmp(type,'eRectangle')
                    obj.clean_params.path_type = 'n';
                    obj.clean_params.camera_ori = 'top';
                    obj.clean_params.dis_trans = 0.1;
                    obj.clean_params.pitch_angle = deg2rad([30,50]);
                    obj.clean_params.yaw_angle = [0,0];
                    obj.clean_params.clean_tool = [0.25,0];
                    obj.clean_params.trans_angle = deg2rad(30);
                elseif strcmp(type,'eCircle')
                    obj.clean_params.path_type = 'n';
                    obj.clean_params.camera_ori = 'top';
                    obj.clean_params.dis_trans = 0.1;
                    obj.clean_params.pitch_angle = deg2rad([30,50]);
                    obj.clean_params.yaw_angle = [0,0];
                    obj.clean_params.clean_tool = [0.25,0];
                    obj.clean_params.trans_angle = deg2rad(30);
                    obj.CalcRotEllipse(vertices);
                    obj.CalcCircleParams(vertices);
                elseif strcmp(type,'eEllipse')
                    obj.clean_params.path_type = 'n';
                    obj.clean_params.camera_ori = 'top';
                    obj.clean_params.dis_trans = 0.1;
                    obj.clean_params.pitch_angle = deg2rad([30,50]);
                    obj.clean_params.yaw_angle = [0,0];
                    obj.clean_params.clean_tool = [0.25,0];
                    obj.clean_params.trans_angle = deg2rad(30);
                    obj.CalcRotEllipse(vertices);
                    obj.CalcEllipseParams(vertices);
                elseif strcmp(type,'eRunway')
                    obj.clean_params.path_type = 'n';
                    obj.clean_params.camera_ori = 'top';
                    obj.clean_params.dis_trans = 0.1;
                    obj.clean_params.pitch_angle = deg2rad([30,50]);
                    obj.clean_params.yaw_angle = [0,0];
                    obj.clean_params.clean_tool = [0.25,0];
                    obj.clean_params.trans_angle = deg2rad(30);
                    obj.CalcRotRunway(vertices);
                elseif strcmp(type,'eOctagon')
                    obj.clean_params.path_type = 'n';
                    obj.clean_params.camera_ori = 'top';
                    obj.clean_params.dis_trans = 0.1;
                    obj.clean_params.pitch_angle = deg2rad([30,50]);
                    obj.clean_params.yaw_angle = [0,0];
                    obj.clean_params.clean_tool = [0.25,0];
                    obj.clean_params.trans_angle = deg2rad(30);
                end
            end
        end

        function [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = PlanCleanPath(obj)
            switch obj.type
            case 'eRectangle'
                rect_planner = QuadranglePlanner;
                via_posrpy_middle = rect_planner.PlanMirror(obj.vertices,obj.hori_times,obj.clean_params);
                via_posrpy_up = [];
                via_posrpy_down = [];
            case 'eCircle'
                [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = obj.PlanCircleMirror();
            case 'eEllipse'
                [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = obj.PlanEllipseMirror();
            case 'eRunway'
                [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = obj.PlanRunwayMirror();
            case 'eOctagon'
                [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = obj.PlanOctagonMirror();
            end
        end
        
        %% A specific planning method to clean circle mirror
        function [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = PlanCircleMirror(obj)
            rect_planner = QuadranglePlanner;
            % plan for middle zone
            truncation_width = 0.4;
            tmp_x = 0.5*truncation_width;
            tmp_y = sqrt(obj.circle_params.radius^2-tmp_x^2);
            vertices_rect(:,1) = obj.circle_params.origin+obj.rot_plane*[tmp_x;-tmp_y;0];
            vertices_rect(:,2) = obj.circle_params.origin+obj.rot_plane*[-tmp_x;-tmp_y;0];
            vertices_rect(:,3) = obj.circle_params.origin+obj.rot_plane*[-tmp_x;tmp_y;0];
            vertices_rect(:,4) = obj.circle_params.origin+obj.rot_plane*[tmp_x;tmp_y;0];
            via_posrpy_rect = rect_planner.UniversalPlan(vertices_rect,obj.clean_params.clean_tool,...
                                    obj.clean_params.pitch_angle,obj.clean_params.yaw_angle,...
                                    obj.clean_params.dis_trans,obj.clean_params.camera_ori,...
                                    obj.clean_params.path_type,obj.clean_params.trans_angle);
            arc_up = vertices_rect(:,4)-0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1);
            arc_middle = obj.vertices(:,1)-0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1);
            arc_down = vertices_rect(:,1)-0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1);
            via_posrpy_right(1:3,2:4) = [arc_up,arc_middle,arc_down];
            via_posrpy_right(1:3,1) = arc_up+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            via_posrpy_right(1:3,5) = arc_down+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            rpy_arc = via_posrpy_rect(4:6,2);
            via_posrpy_right(4:6,:) = [rpy_arc,rpy_arc,rpy_arc,rpy_arc,rpy_arc];
            arc_up = vertices_rect(:,3)+0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1);
            arc_middle = obj.vertices(:,3)+0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1);
            arc_down = vertices_rect(:,2)+0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1);
            via_posrpy_left(1:3,2:4) = [arc_up,arc_middle,arc_down];
            via_posrpy_left(1:3,1) = arc_up+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            via_posrpy_left(1:3,5) = arc_down+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            via_posrpy_left(4:6,:) = [rpy_arc,rpy_arc,rpy_arc,rpy_arc,rpy_arc];
            via_posrpy_middle = [via_posrpy_right,via_posrpy_rect,via_posrpy_left];
            % plan for up zone
            arc_left = vertices_rect(:,3)-0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,2);
            arc_up = obj.vertices(:,4)-0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,2);
            arc_right = vertices_rect(:,4)-0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,2);
            via_posrpy_up(1:3,2:4) = [arc_left,arc_up,arc_right];
            via_posrpy_up(1:3,1) = arc_left+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            via_posrpy_up(1:3,5) = arc_right+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            rpy_arc = tr2rpy(obj.rot_plane*[0,-1,0;-1,0,0;0,0,-1]*rotx(-50),'xyz');
            via_posrpy_up(4:6,:) = [rpy_arc', rpy_arc', rpy_arc', rpy_arc', rpy_arc'];
            % plan for down zone
            arc_left = vertices_rect(:,2)+0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,2);
            arc_down = obj.vertices(:,2)+0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,2);
            arc_right = vertices_rect(:,1)+0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,2);
            via_posrpy_down(1:3,2:4) = [arc_left,arc_down,arc_right];
            via_posrpy_down(1:3,1) = arc_left+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            via_posrpy_down(1:3,5) = arc_right+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            via_posrpy_down(4:6,:) = [rpy_arc', rpy_arc', rpy_arc', rpy_arc', rpy_arc'];
        end
        
        function CalcCircleParams(obj,vertices)
            obj.circle_params.origin = 0.5*(0.5*(vertices(:,1)+vertices(:,3))...
                                        +0.5*(vertices(:,2)+vertices(:,4)));
            obj.circle_params.radius = 0.25*(norm(vertices(:,1)-obj.circle_params.origin)...
                                        +norm(vertices(:,2)-obj.circle_params.origin)...
                                        +norm(vertices(:,3)-obj.circle_params.origin)...
                                        +norm(vertices(:,4)-obj.circle_params.origin));
        end
        
        %% A specific planning method to clean ellipse mirror
        function [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = PlanEllipseMirror(obj)
            rect_planner = QuadranglePlanner;
            % plan for middle zone
            truncation_width = 0.4;
            tmp_x = 0.5*truncation_width;
            tmp_y = sqrt(obj.ellipse_params.a^2*(1-tmp_x^2/obj.ellipse_params.b^2));
            vertices_rect(:,1) = obj.ellipse_params.origin+obj.rot_plane*[tmp_x;-tmp_y;0];
            vertices_rect(:,2) = obj.ellipse_params.origin+obj.rot_plane*[-tmp_x;-tmp_y;0];
            vertices_rect(:,3) = obj.ellipse_params.origin+obj.rot_plane*[-tmp_x;tmp_y;0];
            vertices_rect(:,4) = obj.ellipse_params.origin+obj.rot_plane*[tmp_x;tmp_y;0];
            via_posrpy_rect = rect_planner.UniversalPlan(vertices_rect,obj.clean_params.clean_tool,...
                                    obj.clean_params.pitch_angle,obj.clean_params.yaw_angle,...
                                    obj.clean_params.dis_trans,obj.clean_params.camera_ori,...
                                    obj.clean_params.path_type,obj.clean_params.trans_angle);
            tmp_rpy = via_posrpy_rect(4:6,2);
            via_pos_right = obj.vertices(:,1)-obj.rot_plane(:,1)*0.5*obj.clean_params.clean_tool(1);
            via_pos_left = obj.vertices(:,3)+obj.rot_plane(:,1)*0.5*obj.clean_params.clean_tool(1);
            via_posrpy_right = [via_posrpy_rect(:,1:2),[via_pos_right;tmp_rpy],via_posrpy_rect(:,3:4)];
            via_posrpy_left = [via_posrpy_rect(:,end-3:end-2),[via_pos_left;tmp_rpy],via_posrpy_rect(:,end-1:end)];
            via_posrpy_middle = [via_posrpy_right, via_posrpy_rect, via_posrpy_left];
            % plan for up zone
            r = 0.5*(norm(vertices_rect(:,3)-obj.ellipse_params.origin)...
                +norm(vertices_rect(:,4)-obj.ellipse_params.origin));
            arc_up = obj.ellipse_params.origin+obj.rot_plane(:,2)*r;
            vec_tmp = obj.ellipse_params.origin-vertices_rect(:,3);
            arc_left = vertices_rect(:,3)...
                    +0.5*obj.clean_params.clean_tool(1)*vec_tmp/norm(vec_tmp);
            vec_tmp = obj.ellipse_params.origin-arc_up;
            arc_up = arc_up...
                    +0.5*obj.clean_params.clean_tool(1)*vec_tmp/norm(vec_tmp);
            vec_tmp = obj.ellipse_params.origin-vertices_rect(:,4);
            arc_right = vertices_rect(:,4)...
                    +0.5*obj.clean_params.clean_tool(1)*vec_tmp/norm(vec_tmp);
            via_posrpy_up(1:3,2:4) = [arc_left, arc_up, arc_right];
            via_posrpy_up(1:3,1) = vertices_rect(:,3)+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            via_posrpy_up(1:3,5) = vertices_rect(:,4)+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            rpy_arc = tr2rpy(obj.rot_plane*[0,-1,0;-1,0,0;0,0,-1]*rotx(-50),'xyz');
            via_posrpy_up(4:6,:) = [rpy_arc',rpy_arc',rpy_arc',rpy_arc',rpy_arc'];
            % plan for down zone
            r = 0.5*(norm(vertices_rect(:,1)-obj.ellipse_params.origin)...
                +norm(vertices_rect(:,2)-obj.ellipse_params.origin));
            arc_down = obj.ellipse_params.origin-obj.rot_plane(:,2)*r;
            vec_tmp = obj.ellipse_params.origin-vertices_rect(:,2);
            arc_left = vertices_rect(:,2)+0.5*obj.clean_params.clean_tool(1)*vec_tmp/norm(vec_tmp);
            vec_tmp = obj.ellipse_params.origin-arc_down;
            arc_down = arc_down+0.5*obj.clean_params.clean_tool(1)*vec_tmp/norm(vec_tmp);
            vec_tmp = obj.ellipse_params.origin-vertices_rect(:,1);
            arc_right = vertices_rect(:,1)+0.5*obj.clean_params.clean_tool(1)*vec_tmp/norm(vec_tmp);
            via_posrpy_down(1:3,2:4) = [arc_left, arc_down, arc_right];
            via_posrpy_down(1:3,1) = vertices_rect(:,2)+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            via_posrpy_down(1:3,5) = vertices_rect(:,1)+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            rpy_arc = tr2rpy(obj.rot_plane*[0,-1,0;-1,0,0;0,0,-1]*rotx(-50),'xyz');
            via_posrpy_down(4:6,:) = [rpy_arc',rpy_arc',rpy_arc',rpy_arc',rpy_arc'];
        end

        function CalcRotEllipse(obj,vertices)
            origin = 0.5*(0.5*(vertices(:,1)+vertices(:,3))+0.5*(vertices(:,2)+vertices(:,4)));
            x0_ellipse = vertices(:,1)-origin;
            z0_ellipse = cross(x0_ellipse,vertices(:,4)-vertices(:,2));
            y0_ellipse = cross(z0_ellipse,x0_ellipse);
            obj.rot_plane = [x0_ellipse/norm(x0_ellipse),...
                            y0_ellipse/norm(y0_ellipse),...
                            z0_ellipse/norm(z0_ellipse)];
        end

        function CalcEllipseParams(obj,vertices)
            obj.ellipse_params.origin = 0.5*(0.5*(vertices(:,1)+vertices(:,3))...
                                        +0.5*(vertices(:,2)+vertices(:,4)));
            obj.ellipse_params.a = 0.5*(norm(obj.ellipse_params.origin-vertices(:,2))...
                                        +norm(obj.ellipse_params.origin-vertices(:,4)));
            obj.ellipse_params.b = 0.5*(norm(obj.ellipse_params.origin-vertices(:,1))...
                                        +norm(obj.ellipse_params.origin-vertices(:,3)));
        end
        
        %% A specific planning method to clean runway mirror
        function [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = PlanRunwayMirror(obj)
            rect_planner = QuadranglePlanner;
            % plan for middle zone
            via_posrpy_middle = rect_planner.UniversalPlan(obj.vertices,obj.clean_params.clean_tool,...
                                    obj.clean_params.pitch_angle,obj.clean_params.yaw_angle,...
                                    obj.clean_params.dis_trans,obj.clean_params.camera_ori,...
                                    obj.clean_params.path_type,obj.clean_params.trans_angle);
            % plan for up zone
            r = 0.5*norm(obj.vertices(:,3)-obj.vertices(:,4));
            origin = 0.5*(obj.vertices(:,3)+obj.vertices(:,4));
            arc_up = origin+r*obj.rot_plane(:,1);
            arc_up = arc_up-0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1);
            via_posrpy_up(1:3,2:4) = [obj.vertices(:,3)-0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1),...
                                        arc_up,obj.vertices(:,4)-0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1)];
            via_posrpy_up(1:3,1) = obj.vertices(:,3)+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            via_posrpy_up(1:3,5) = obj.vertices(:,4)+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            rpy_arc = tr2rpy(obj.rot_plane*[-1,0,0;0,1,0;0,0,-1]*rotx(-50), 'xyz');
            via_posrpy_up(4:6,:) = [rpy_arc',rpy_arc',rpy_arc',rpy_arc',rpy_arc'];
            % plan for down zone
            r = 0.5*norm(obj.vertices(:,1)-obj.vertices(:,2));
            origin = 0.5*(obj.vertices(:,1)+obj.vertices(:,2));
            arc_down = origin-r*obj.rot_plane(:,1);
            arc_down = arc_down+0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1);
            via_posrpy_down(1:3,2:4) = [obj.vertices(:,2)+0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1),...
                                        arc_down,obj.vertices(:,1)+0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1)];
            via_posrpy_down(1:3,1) = obj.vertices(:,2)+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            via_posrpy_down(1:3,5) = obj.vertices(:,1)+obj.clean_params.dis_trans*obj.rot_plane(:,3);
            rpy_arc = tr2rpy(obj.rot_plane*[-1,0,0;0,1,0;0,0,-1]*rotx(-50), 'xyz');
            via_posrpy_down(4:6,:) = [rpy_arc',rpy_arc',rpy_arc',rpy_arc',rpy_arc'];
        end

        function CalcRotRunway(obj,vertices)
            x0_runway = vertices(:,4)-vertices(:,1);
            z0_runway = cross(vertices(:,4)-vertices(:,2),vertices(:,3)-vertices(:,1));
            y0_runway = cross(z0_runway,x0_runway);
            obj.rot_plane = [x0_runway/norm(x0_runway),y0_runway/norm(y0_runway),z0_runway/norm(z0_runway)];
        end
        
        %% A specific planning method to clean octagon mirror
        function [via_posrpy_up,via_posrpy_middle,via_posrpy_down] = PlanOctagonMirror(obj)
            rect_planner = QuadranglePlanner;
            % plan for up zone
            rect_vertices = [obj.vertices(:,1),obj.vertices(:,3:5)];
            via_posrpy_middle = rect_planner.UniversalPlan(rect_vertices,obj.clean_params.clean_tool,...
                                obj.clean_params.pitch_angle,obj.clean_params.yaw_angle,...
                                obj.clean_params.dis_trans,obj.clean_params.camera_ori,...
                                obj.clean_params.path_type,obj.clean_params.trans_angle);
            obj.rot_plane = rect_planner.rot_plane;
            % plan for down zone
            pos_tmp1 = obj.vertices(:,3)+0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1);
            pos_tmp2 = obj.vertices(:,2)+0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1);
            pos_left = [pos_tmp1+obj.clean_params.dis_trans*obj.rot_plane(:,3), pos_tmp1,...
                            pos_tmp2, pos_tmp2+obj.clean_params.dis_trans*obj.rot_plane(:,3)];
            rpy_left = tr2rpy(obj.rot_plane*[-1,0,0;0,1,0;0,0,-1]*rotx(-50),'xyz');
            rpy_tmp = tr2rpy([0,0,1;1,0,0;0,1,0], 'xyz');
            posrpy_left = [pos_left; rpy_left',rpy_left',rpy_left',rpy_tmp'];
            pos_tmp1 = obj.vertices(:,1)+0.5*obj.clean_params.clean_tool(1)*obj.rot_plane(:,1);
            pos_right = [pos_tmp1+obj.clean_params.dis_trans*obj.rot_plane(:,3), pos_tmp1,...
                                pos_tmp2, pos_tmp2+obj.clean_params.dis_trans*obj.rot_plane(:,3)];
            rpy_right = tr2rpy(obj.rot_plane*[-1,0,0;0,1,0;0,0,-1]*rotx(50),'xyz');
            posrpy_right = [pos_right; rpy_right',rpy_right',rpy_right',rpy_tmp'];
            via_posrpy_down = [posrpy_left, posrpy_right];
            via_posrpy_up = [];
        end
        
    
    end
    
    
end
