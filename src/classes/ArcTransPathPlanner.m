classdef ArcTransPathPlanner < handle
% plan a continuous path according to the given points
% arc transits between 2 lines
% ref: https://blog.csdn.net/qq_26565435/article/details/98789361
% ref: Robotics: Modelling, Plannig and Control, chapter 4.3
    
    properties
        center
        radius
        theta
        rot
        dr
        dl
        pt
        line_vec
        distance
        dis_interval
    
        nump
        numarc
        p_initial
        p_goal
        trans_type
    end
    
    
    methods
        %% Class Constructor
        function obj = ArcTransPathPlanner(pos, radius)
            obj.p_initial = pos(:,1);
            obj.p_goal = pos(:,end);
            obj.nump = size(pos, 2);
            if radius>0
                obj.numarc = obj.nump-2;
                obj.radius = radius;
                obj.CalcArcInfo(pos);
                obj.trans_type = 'arc';
            else
                obj.numarc = obj.nump/2-1;
                obj.radius = 0.5*norm(pos(:,3)-pos(:,2));
                obj.CalcSemicircleInfo(pos);
                obj.trans_type = 'semicircle';
            end
            obj.dis_interval = obj.CalcDisInsterval();
        end
        
        %% Calculate Arc Transition
        function CalcArcInfo(obj, pos)
            for idx=1:obj.numarc
                [obj.center(:,idx), obj.theta(idx), pt1, pt2] = obj.CalcArcPoints(pos(:,idx), pos(:,idx+1), pos(:,idx+2));
                obj.pt(:,2*idx-1) = pt1;
                obj.pt(:,2*idx) = pt2;
                obj.dr(idx) = obj.radius*(pi-obj.theta(idx));
                obj.dl(idx) = norm(pt1-pos(:,idx));
                obj.rot{idx} = obj.CalcArcRot(obj.center(:,idx), pt1, pt2);
                if idx ~=1
                    obj.dl(idx) = obj.dl(idx)-norm(pos(:,idx)-obj.pt(:,2*(idx-1)));
                end
                if idx==obj.numarc
                    obj.dl(idx+1) = norm(pos(:,end)-pt2);
                end
                obj.line_vec(:,idx) = (pos(:,idx+1)-pos(:,idx))/norm(pos(:,idx+1)-pos(:,idx));
            end
            obj.line_vec(:,obj.numarc+1) = (pos(:,end)-pos(:,end-1))/norm(pos(:,end)-pos(:,end-1));
            obj.distance = sum(obj.dl)+sum(obj.dr);
        end
        
        function [center, theta, pt1, pt2] = CalcArcPoints(obj, p1, p2, p3)
            p21 = p1-p2; p23 = p3-p2;
            tmp_cos = dot(p21, p23)/norm(p21)/norm(p23);
            theta = acos(tmp_cos);
            
            p2t1 = obj.radius/tan(0.5*theta)*p21/norm(p21);
            pt1 = p2+p2t1;
            p2t2 = obj.radius/tan(0.5*theta)*p23/norm(p23);
            pt2 = p2+p2t2;
            
            pt1t2 = pt2-pt1;
            pt1m1 = 0.5*pt1t2;
            p2m1 = p2t1+pt1m1;
            p2c = obj.radius/sin(0.5*theta)/norm(p2m1)*p2m1;
            center = p2+p2c;            
        end
        
        %% Calculate Semicircle Transition
        function CalcSemicircleInfo(obj, pos)
            for idx=1:obj.numarc
                p1 = pos(:,2*idx-1); p2 = pos(:,2*idx); p3 = pos(:,2*idx+1); p4 = pos(:,2*idx+2);
                [obj.center(:,idx), pt1, pt2] = obj.CalcSemicirclePoints(p1,p2,p3,p4);
                obj.pt(:,2*idx-1) = pt1;
                obj.pt(:,2*idx) = pt2;
                obj.dr(idx) = pi*obj.radius;
                obj.rot{idx} = obj.CalcArcRot(obj.center(:,idx), pt1, pos(:,2*idx));
                if idx~=1
                    obj.dl(idx) = norm(obj.pt(:,2*idx-1)-obj.pt(:,2*idx-2));
                else
                    obj.dl(idx) = norm(pos(:,1)-obj.pt(:,1));
                end
                obj.line_vec(:,idx) = (pos(:,2*idx)-pos(:,2*idx-1))/norm(pos(:,2*idx)-pos(:,2*idx-1));
            end
            obj.dl(obj.numarc+1) = norm(pos(:,end)-obj.pt(:,end));
            obj.line_vec(:,obj.numarc+1) = (pos(:,end)-pos(:,end-1))/norm(pos(:,end)-pos(:,end-1));
            obj.distance = sum(obj.dl)+sum(obj.dr);
        end
        
        function [center, pt1, pt2] = CalcSemicirclePoints(obj, p1, p2, p3, p4)
            p21 = p1-p2; p34 = p4-p3;
            p2t1 = obj.radius*p21/norm(p21);
            p3t2 = obj.radius*p34/norm(p34);
            pt1 = p2+p2t1;
            pt2 = p3+p3t2;
            center = 0.5*(pt1+pt2);
        end
        
        %% Useful and Universal Functions
        function rot = CalcArcRot(obj, center, p1, p2)
            px = p1-center;
            n = px/norm(px);
            a = cross(px, (p2-center));
            a = a/norm(a);
            o = cross(a, n);
            rot = [n, o, a];
        end

        function dis = CalcDisInsterval(obj)
            dis = zeros(1,length(obj.dl)+length(obj.dr)+1);
            m = dis(1);
            for idx=2:length(dis)
                if mod(idx,2)==0
                    m = m+obj.dl(idx/2);
                else
                    m = m+obj.dr((idx-1)/2);
                end
                dis(idx) = m;
            end
        end
        
        function idx = CalcPosidx(obj, varp)
            if varp>obj.dis_interval(end)
                idx = length(obj.dis_interval)-1;
            elseif varp<obj.dis_interval(1)
                idx = 1;
            else
                idx = discretize(varp, obj.dis_interval);
            end
        end            
        
        %% Generate and Plot Trajectory 
        function p = GeneratePath(obj, varp)
            idx = obj.CalcPosidx(varp);
            if mod(idx,2)==1
                if idx==1
                    p = obj.p_initial+obj.line_vec(:,idx)*varp;
                else
                    p = obj.pt(:,idx-1)+obj.line_vec(:,(idx+1)/2)*(varp-obj.dis_interval(idx));
                end
            else
                th = (varp-obj.dis_interval(idx))/obj.radius;
                parc = zeros(3,1);
                parc(1) = obj.radius*cos(th);
                parc(2) = obj.radius*sin(th);
                p = obj.center(:,idx/2)+obj.rot{idx/2}*parc;
            end
        end
        
        function [p, v, a] = GenerateMotion(obj, varp, varv, vara)
            p = obj.GeneratePath(varp);
            idx = obj.CalcPosidx(varp);
            if mod(idx,2)==1
                v = varv*obj.line_vec(:,(idx+1)/2);
                a = vara*obj.line_vec(:,(idx+1)/2);
            else
                s = varp-obj.dis_interval(idx);
                r = obj.radius;
                vc=zeros(3,1); ac = zeros(3,1);
                vc(1) = -varv*sin(s/r);
                vc(2) = varv*cos(s/r);
                v = obj.rot{idx/2}*vc;
                ac(1) = -varv^2*cos(s/r)/r-vara*sin(s/r);
                ac(2) = -varv^2*sin(s/r)/r+vara*cos(s/r);
                a = obj.rot{idx/2}*ac;
            end
            v = [v; norm(v)]; a = [a;norm(a)];
        end
        
        function [pos, vel, acc] = GenerateTraj(obj, varp, varv, vara)
            pos = []; vel = []; acc = [];
            for idx=1:length(varp)
                [p, v, a] = obj.GenerateMotion(varp(idx), varv(idx), vara(idx));
                pos = [pos, p];
                vel = [vel, v];
                acc = [acc, a];
            end
        end
        
        function PlotTraj(obj, varp, varv, vara, tf, dt)
            [pos, vel, acc] = obj.GenerateTraj(varp, varv, vara);
            str_pos = {'px', 'py', 'pz'};
            str_vel = {'vx', 'vy', 'vz', '|v|'};
            str_acc = {'ax', 'ay', 'az', '|a|'};
            time=0:dt:tf;
            figure
            for idx=1:3
                subplot(3,1,idx); plot(time, pos(idx,:), 'k-'); grid on; ylabel(str_pos{idx});
            end
            figure
            for idx=1:4
                subplot(4,2,2*idx-1); plot(time, vel(idx,:), 'k-'); grid on; ylabel(str_vel{idx});
                subplot(4,2,2*idx); plot(time, acc(idx,:), 'k-'); grid on; ylabel(str_acc{idx});
            end    
        end
        
    end
      
end