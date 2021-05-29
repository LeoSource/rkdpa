%   use rpy to represent rotation

%   TO DO: add quaternion and aixs-angle methods
%   Author:
%   liao zhixiang, zhixiangleo@163.com

classdef LineTrajPlanner < handle

    properties
        pos_initial
        rpy_initial
        pos_dir
        rot_dir
        tf_pos
        tf_rot
        pos_uplanner
        rot_uplanner
        
        option
    end

    methods
        function obj = LineTrajPlanner(pos0, posn,line_vmax,line_amax,pduration,pvel_cons,...
                                        rpy0,rpyn,ang_vmax,ang_amax,rduration,rvel_cons,opt)
            obj.option = opt;
            obj.pos_initial = pos0;
            obj.rpy_initial = rpy0;
            if strcmp(opt,'both')
                obj.InitPosPlanner(pos0,posn,line_vmax,line_amax,pduration,pvel_cons);
                obj.InitRotPlanner(rpy0,rpyn,ang_vmax,ang_amax,rduration,rvel_cons);
                obj.tf_pos = obj.pos_uplanner.tf;
                obj.tf_rot = obj.rot_uplanner.tf;
            elseif strcmp(opt, 'pos')
                obj.InitPosPlanner(pos0,posn,line_vmax,line_amax,pduration,pvel_cons);
                obj.tf_pos = obj.pos_uplanner.tf;
            elseif strcmp(opt, 'rot')
                obj.InitRotPlanner(rpy0,rpyn,ang_vmax,ang_amax,rduration,rvel_cons);
                obj.tf_rot = max([obj.roll_planner.tf,obj.pitch_planner.tf,obj.yaw_planner.tf]);
            else
                error('error line option');
            end
        end

        function InitPosPlanner(obj, pos0, posn, line_vmax, line_amax, tf, vel_cons)
            line_len = norm(posn-pos0);
            obj.pos_dir = (posn-pos0)/line_len;
            if isempty(tf)
                obj.pos_uplanner = LspbTrajPlanner([0,line_len], line_vmax, line_amax,[],vel_cons);
            else
                obj.pos_uplanner = LspbTrajPlanner([0,line_len], line_vmax, line_amax,tf);
            end
        end

        function InitRotPlanner(obj, rpy0, rpyn, ang_vmax, ang_amax, tf,vel_cons)
            rpy_len = norm(rpyn-rpy0);
            obj.rot_dir = (rpyn-rpy0)/rpy_len;
            if isempty(tf)
                obj.rot_uplanner = LspbTrajPlanner([0,rpy_len], ang_vmax, ang_amax, [], vel_cons);
            else
                obj.rot_uplanner = LspbTrajPlanner([0,rpy_len], ang_vmax, ang_amax, tf);
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
            [up,~,~] = obj.rot_uplanner.GenerateMotion(t);
            rpy = obj.rpy_initial+up*obj.rot_dir;
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