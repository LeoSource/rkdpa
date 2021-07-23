classdef ArcPlanner < handle

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
        tf
        pos_uplanner
        rot_uplanner

        option
        arc_style
    end


    methods
        %% initialize planner
        function obj = ArcPlanner(pos1,pos2,pos3,line_vmax,line_amax,pvel_cons,...
                                        rpy0,rpyn,ang_vmax,ang_amax,rvel_cons,style)
            obj.CalcTrajOption([pos1;rpy0], [pos3;rpyn]);
            obj.arc_style = style;
            obj.pos_initial = pos1;
            obj.rpy_initial = rpy0;
            if strcmp(obj.option, 'both')
                obj.InitPosPlanner(pos1,pos2,pos3,line_vmax,line_amax,[],pvel_cons);
                obj.InitRotPlanner(rpy0,rpyn,ang_vmax,ang_amax,[],rvel_cons);
                obj.tf_pos = obj.pos_uplanner.tf;
                obj.tf_rot = obj.rot_uplanner.tf;
                obj.tf = max([obj.tf_pos,obj.tf_rot]);
                if obj.tf_pos>obj.tf_rot
                    obj.InitRotPlanner(rpy0,rpyn,ang_vmax,ang_amax,obj.tf,rvel_cons);
                else
                    obj.InitPosPlanner(pos1,pos2,pos3,line_vmax,line_amax,obj.tf,pvel_cons);
                end
            elseif strcmp(obj.option, 'pos')
                obj.InitPosPlanner(pos1,pos2,pos3,line_vmax,line_amax,[],pvel_cons);
                obj.tf_pos = obj.pos_uplanner.tf;
                obj.tf = obj.tf_pos;
            elseif strcmp(obj.option, 'rot')
                obj.InitRotPlanner(rpy0,rpyn,ang_vmax,ang_amax,[],rvel_cons);
                obj.tf_rot = obj.rot_uplanner.tf;
                obj.tf = obj.tf_rot;
            else
                error('error line option');
            end
        end

        function InitPosPlanner(obj,pos1,pos2,pos3,line_vmax,line_amax,tf,vel_cons)
            obj.InitArcInfo(pos1,pos2,pos3);
            if isempty(tf)
                obj.pos_uplanner = LspbPlanner([0,obj.pos_len],line_vmax,line_amax,[],vel_cons);
            else
                obj.pos_uplanner = LspbPlanner([0,obj.pos_len],line_vmax,line_amax,tf,vel_cons);
            end
        end
        
        function InitArcInfo(obj,pos1,pos2,pos3)
            if strcmp(obj.arc_style,'arctrans')
                p2p1 = pos1-pos2; p2p3 = pos3-pos2;
                inc_angle = acos(dot(p2p1,p2p3)/norm(p2p1)/norm(p2p3));
                obj.theta = pi-inc_angle;
                obj.radius = norm(p2p1)*tan(0.5*inc_angle);
                pc = 0.5*(pos1+pos3); p2pc = pc-pos2;
                scale = obj.radius/sin(0.5*inc_angle)/norm(p2pc);
                p2center = scale*p2pc;
                obj.center = pos2+p2center;
                n = (pos1-obj.center)/norm(pos1-obj.center);
                [a1, b1, c1, ~] = obj.PointsCoplane(pos1, pos2, pos3);
                a = [a1; b1; c1]/norm([a1; b1; c1]);
                o = cross(a,n);
                obj.rot = [n, o, a];
                obj.pos_len = obj.radius*obj.theta;
            elseif strcmp(obj.arc_style,'arc')
                [a1, b1, c1, d1] = obj.PointsCoplane(pos1, pos2, pos3);
                [a2, b2, c2, d2] = obj.RadiusEqual(pos1, pos2);
                [a3, b3, c3, d3] = obj.RadiusEqual(pos1, pos3);
                center_tmp(1) = -(b1*c2*d3-b1*c3*d2-b2*c1*d3+b2*c3*d1+b3*c1*d2-b3*c2*d1)/...
                                    (a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
                center_tmp(2) = (a1*c2*d3-a1*c3*d2-a2*c1*d3+a2*c3*d1+a3*c1*d2-a3*c2*d1)/...
                                    (a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
                center_tmp(3) = -(a1*b2*d3-a1*b3*d2-a2*b1*d3+a2*b3*d1+a3*b1*d2-a3*b2*d1)/...
                                    (a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1);
                obj.center = reshape(center_tmp, 3,1);
                tmp_value = (pos1(1)-obj.center(1))^2+(pos1(2)-obj.center(2))^2+(pos1(3)-obj.center(3))^2;
                obj.radius = sqrt(tmp_value);
                line_length = norm(pos3-pos1);
                tmp_cos = (obj.radius^2+obj.radius^2-line_length^2)/(2*obj.radius*obj.radius);
                obj.theta = acos(tmp_cos);
                n = (pos1-obj.center)/norm(pos1-obj.center);
                n = reshape(n, 3,1);
                a = [a1; b1; c1]/norm([a1; b1; c1]);
                o = cross(a,n);
                obj.rot = [n, o, a];
                obj.pos_len = obj.radius*obj.theta;
            end
            
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
        
        function [a, b, c, d] = RadiusEqual(obj, pos1, pos2)
            a = 2*(pos2(1)-pos1(1));
            b = 2*(pos2(2)-pos1(2));
            c = 2*(pos2(3)-pos1(3));
            d = pos1(1)^2+pos1(2)^2+pos1(3)^2-...
                pos2(1)^2-pos2(2)^2-pos2(3)^2;
        end

        function InitRotPlanner(obj, rpy0, rpyn, ang_vmax, ang_amax, tf,vel_cons)
            delta_rot = rpy2r(180/pi*rpyn','xyz')*rpy2r(180/pi*rpy0','xyz')';
            [rpy_len, obj.rot_dir] = tr2angvec(delta_rot);
            obj.rot_dir = reshape(obj.rot_dir, 3,1);
            obj.rot_len = rpy_len;
            if isempty(tf)
                obj.rot_uplanner = LspbPlanner([0,rpy_len], ang_vmax, ang_amax, [], vel_cons);
            else
                obj.rot_uplanner = LspbPlanner([0,rpy_len], ang_vmax, ang_amax, tf, vel_cons);
            end
        end
        
        function CalcTrajOption(obj, pos_rpy1, pos_rpy2)
            pos_length = norm(pos_rpy1(1:3)-pos_rpy2(1:3));
            %there is unit wrong with rpy2r function
            delta_rot = rpy2r(180/pi*pos_rpy2(4:6)','xyz')*rpy2r(180/pi*pos_rpy1(4:6)','xyz')';
            [rpy_length, ~] = tr2angvec(delta_rot);
            if pos_length>1e-5 && rpy_length>1e-5
                obj.option = "both";
            elseif pos_length>1e-5 && rpy_length<=1e-5
                obj.option = "pos";
            elseif pos_length<=1e-5 && rpy_length>1e-5
                obj.option = "rot";
            else
                obj.option = "none";
            end
        end

        %% generate trajectory
        function [pos,pvel,pacc,rpy,rvel,racc] = GenerateTraj(obj,dt)
            pos = []; pvel = []; pacc = [];
            rpy = []; rvel = []; racc = [];
            for t=0:dt:obj.tf
                [p,vp,ap,r,vr,ar] = obj.GenerateMotion(t);
                pos = [pos,p]; pvel = [pvel,vp]; pacc = [pacc,ap];
                rpy = [rpy,r]; rvel = [rvel,vr]; racc = [racc,ar];
            end
        end

        function [p,vp,ap,r,vr,ar] = GenerateMotion(obj,t)
            if strcmp(obj.option,'pos')
                [p,vp,ap] = obj.GeneratePosMotion(t);
                r = obj.rpy_initial;
                vr = zeros(3,1);
                ar = zeros(3,1);
            elseif strcmp(obj.option,'rot')
                [r,vr,ar] = obj.GenerateRotMotion(t);
                p = obj.pos_initial;
                vp = zeros(3,1);
                ap = zeros(3,1);
            elseif strcmp(obj.option,'both')
                [p,vp,ap] = obj.GeneratePosMotion(t);
                [r,vr,ar] = obj.GenerateRotMotion(t);
            end
        end

        function [p,vp,ap] = GeneratePosMotion(obj,t)
            if t>obj.tf
                up = obj.pos_len;
                uv = 0;
                ua = 0;
            else
                [up,uv,ua] = obj.pos_uplanner.GenerateMotion(t);
            end
            r = obj.radius;
            th = up/r;
            parc = zeros(3,1);
            parc(1) = r*cos(th);
            parc(2) = r*sin(th);
            p = obj.center+obj.rot*parc;
            vc = zeros(3,1); ac = zeros(3,1);
            vc(1) = -uv*sin(up/r);
            vc(2) = uv*cos(up/r);
            vp = obj.rot*vc;
            ac(1) = -uv^2*cos(up/r)/r-ua*sin(up/r);
            ac(2) = -uv^2*sin(up/r)/r+ua*cos(up/r);
            ap = obj.rot*ac;
        end

        function [r,vr,ar] = GenerateRotMotion(obj,t)
            if t>obj.tf
                up = obj.rot_len;
                uv = 0;
                ua = 0;
            else
                [up,uv,ua] = obj.rot_uplanner.GenerateMotion(t);
            end
            delta_rot = angvec2r(up, obj.rot_dir);
            cmd_rot = delta_rot*rpy2r(180/pi*obj.rpy_initial', 'xyz');
            r = tr2rpy(cmd_rot, 'xyz')';
            vr = obj.rot_dir*uv;
            ar = obj.rot_dir*ua;
        end

        %% generate path
        function [pos,rpy] = GeneratePath(obj, dt)
            pos = []; rpy = [];
            for t=0:dt:obj.tf
                [p,r] = obj.GeneratePoint(t);
                pos = [pos, p]; rpy = [rpy, r];
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
            delta_rot = angvec2r(up, obj.rot_dir);
            cmd_rot = delta_rot*rpy2r(180/pi*obj.rpy_initial', 'xyz');
            rpy = tr2rpy(cmd_rot, 'xyz')';
        end

    end


    
end