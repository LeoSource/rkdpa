classdef ArcTrajPlanner < handle

    properties
        center
        radius
        theta
        rot

        pos_initial
        rpy_initial
        rot_dir
        pos_len
        rot_len
        tf_pos
        tf_rot
        pos_uplanner
        rot_uplanner

        option
    end


    methods
        function obj = ArcTrajPlanner(pos1,pos2,pos3,line_vmax,line_amax,pduration,pvel_cons,...
                                        rpy0,rpyn,ang_vmax,ang_amax,rduration,rvel_cons,opt)
            obj.option = opt;
            obj.pos_initial = pos1;
            obj.rpy_initial = rpy0;
            if strcmp(opt, 'both')
                obj.InitPosPlanner(pos1,pos2,pos3,line_vmax,line_amax,pduration,pvel_cons);
                obj.InitRotPlanner(rpy0,rpyn,ang_vmax,ang_amax,rduration,rvel_cons);
                obj.tf_pos = obj.pos_uplanner.tf;
                obj.tf_rot = obj.rot_uplanner.tf;
            elseif strcmp(opt, 'pos')
                obj.InitPosPlanner(pos1,pos2,pos3,line_vmax,line_amax,pduration,pvel_cons);
                obj.tf_pos = obj.pos_uplanner.tf;
            elseif strcmp(opt, 'rot')
                obj.InitRotPlanner(rpy0,rpyn,ang_vmax,ang_amax,rduration,rvel_cons);
                obj.tf_rot = obj.rot_uplanner.tf;
            else
                error('error line option');
            end
        end

        function InitPosPlanner(obj,pos1,pos2,pos3,line_vmax,line_amax,tf,vel_cons)
            p2p1 = pos1-pos2; p2p3 = pos3-pos2;
            inc_angle = acos(dot(p2p1,p2p3)/norm(p2p1)/norm(p2p3));
            obj.theta = pi-inc_angle;
            obj.radius = norm(p2p1)*tan(0.5*inc_angle);
            pc = 0.5*(pos1+pos3); p2pc = pc-pos2;
            scale = obj.radius/sin(0.5*inc_angle)/norm(p2pc);
            p2center = scale*p2pc;
            obj.center = pos2+p2center;
            obj.pos_len = obj.radius*obj.theta;
            if isempty(tf)
                obj.pos_uplanner = LspbTrajPlanner([0,obj.pos_len],line_vmax,line_amax,[],vel_cons);
            else
                obj.pos_uplanner = LspbTrajPlanner([0,obj.pos_len],line_vmax,line_amax,tf);
            end

            n = (pos1-obj.center)/norm(pos1-obj.center);
            [a1, b1, c1, ~] = obj.PointsCoplane(pos1, pos2, pos3);
            a = [a1; b1; c1]/norm([a1; b1; c1]);
            o = cross(a,n);
            obj.rot = [n, o, a];

        end
        
        function [a, b, c, d] = PointsCoplane(obj, pos1, pos2, pos3)
            x1 = pos1(1); y1 = pos1(2); z1 = pos1(3);
            x2 = pos2(1); y2 = pos2(2); z2 = pos2(3);
            x3 = pos3(1); y3 = pos3(2); z3 = pos3(3);
            a = y1*z2-y2*z1-y1*z3+y3*z1+y2*z3-y3*z2;
            b = -(x1*z2-x2*z1-x1*z3+x3*z1+x2*z3-x3*z2);
            c = x1*y2-x2*y1-x1*y3+x3*y1+x2*y3-x3*y2;
            d = -(x1*y2*z3-x1*y3*z2-x2*y1*z3+x2*y3*z1+x3*y1*z2-x3*y2*z1);
        end    

        function InitRotPlanner(obj, rpy0, rpyn, ang_vmax, ang_amax, tf,vel_cons)
            rpy_len = norm(rpyn-rpy0);
            obj.rot_len = rpy_len;
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
            if t>obj.tf_pos
                up = obj.pos_len;
            else
                [up,~,~] = obj.pos_uplanner.GenerateMotion(t);
            end
            th = up/obj.radius;
            parc = zeros(3,1);
            parc(1) = obj.radius*cos(th);
            parc(2) = obj.radius*sin(th);
            pos = obj.center+obj.rot*parc;
        end

        function rpy = GenerateRot(obj,t)
            if t>obj.tf_rot
                up = obj.rot_len;
            else
                [up,~,~] = obj.rot_uplanner.GenerateMotion(t);
            end
            rpy = obj.rpy_initial+up*obj.rot_dir;
        end

        function [pos,rpy] = GeneratePath(obj, dt)
            pos = []; rpy = [];
            tf = max([obj.tf_pos,obj.tf_rot]);
            for t=0:dt:tf
                [p,r] = obj.GeneratePoint(t);
                pos = [pos, p]; rpy = [rpy, r];
            end
        end

    end


    
end