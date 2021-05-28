%   use rpy to represent rotation

%   TO DO: add quaternion and aixs-angle methods
%   Author:
%   liao zhixiang, zhixiangleo@163.com

classdef LineTrajPlanner < handle

    properties
        pos_initial
        rpy_initial
        pos_dir
        tf
        pos_uplanner
        rot_uplanner
    end
    properties
        roll_planner
        pitch_planner
        yaw_planner
        option
    end

    methods
        function obj = LineTrajPlanner(pos0, posn,line_vmax,line_amax,...
                                        rpy0,rpyn,ang_vmax,ang_amax,opt)
            obj.option = opt;
            obj.pos_initial = pos0;
            obj.rpy_initial = rpy0;
            if strcmp(opt,'both')
                obj.InitPosPlanner(pos0,posn,line_vmax,line_amax);
                obj.InitRotPlanner(rpy0,rpyn,ang_vmax,ang_amax);
                obj.tf = max([obj.pos_uplanner.tf, obj.pitch_planner.tf,...
                            obj.yaw_planner.tf, obj.roll_planner.tf]);
                obj.InitPosPlanner(pos0,posn,line_vmax,line_amax,obj.tf);
                obj.InitRotPlanner(rpy0,rpyn,ang_vmax,ang_amax,obj.tf);
            elseif strcmp(opt, 'pos')
                obj.InitPosPlanner(pos0,posn,line_vmax,line_amax);
                obj.tf = obj.pos_uplanner.tf;
            elseif strcmp(opt, 'rot')
                obj.InitRotPlanner(rpy0,rpyn,ang_vmax,ang_amax);
                obj.tf = max([obj.roll_planner.tf,obj.pitch_planner.tf,obj.yaw_planner.tf]);
            else
                error('error line option');
            end
        end

        function InitPosPlanner(obj, pos0, posn, line_vmax, line_amax, tf)
            line_len = norm(posn-pos0);
            obj.pos_dir = (posn-pos0)/line_len;
            if nargin==5
                obj.pos_uplanner = LspbTrajPlanner([0,line_len], line_vmax, line_amax);
            elseif nargin==6
                obj.pos_uplanner = LspbTrajPlanner([0,line_len], line_vmax, line_amax,tf);
            end
        end

        function InitRotPlanner(obj, rpy0, rpyn, ang_vmax, ang_amax, tf)
            if nargin==5
                obj.roll_planner = LspbTrajPlanner([rpy0(1), rpyn(1)],ang_vmax(1),ang_amax(1));
                obj.pitch_planner = LspbTrajPlanner([rpy0(2), rpyn(2)],ang_vmax(2),ang_amax(2));
                obj.yaw_planner = LspbTrajPlanner([rpy0(3),rpyn(3)],ang_vmax(3),ang_amax(3));
            elseif nargin==6
                obj.roll_planner = LspbTrajPlanner([rpy0(1), rpyn(1)],ang_vmax(1),ang_amax(1),tf);
                obj.pitch_planner = LspbTrajPlanner([rpy0(2), rpyn(2)],ang_vmax(2),ang_amax(2),tf);
                obj.yaw_planner = LspbTrajPlanner([rpy0(3),rpyn(3)],ang_vmax(3),ang_amax(3),tf);
            end
        end

        function [pos,rpy] = GeneratePoint(obj, t)
            if strcmp(obj.option,'pos')
                pos = obj.GeneratePos(t);
                rpy = obj.rpy_initial;
            elseif strcmp(obj.option,'rot')
                pos = obj.pos_initial;
                rpy = obj.GenerateRot(t);
            elseif strcmp(obj.option,'both')
                pos = obj.GeneratePos(t);
                rpy = obj.GenerateRot(t);
            end
        end

        function pos = GeneratePos(obj,t)
            [up,~,~] = obj.pos_uplanner.GenerateMotion(t);
            pos = obj.pos_initial+up*obj.pos_dir;
        end

        function rpy = GenerateRot(obj,t)
            rpy = zeros(3,1);
            rpy(1) = obj.roll_planner.GenerateMotion(t);
            rpy(2) = obj.pitch_planner.GenerateMotion(t);
            rpy(3) = obj.yaw_planner.GenerateMotion(t);
        end

        function [pos,rpy] = GeneratePath(obj, dt)
            pos = []; rpy = [];
            for t=0:dt:obj.tf
                [p,r] = obj.GeneratePoint(t);
                pos = [pos, p]; rpy = [rpy, r];
            end
        end
    end


end