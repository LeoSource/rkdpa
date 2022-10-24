%   Cartesian Trajectory Planner Class:
%   A specific class that plans and generates line, arc and their hybrid trajectory
%   You can set the conti_type to define Cartesian trajectory:
%   false means that trajectory is consisits of only lines, trajectory will
%   go through all the via-points
%   true means that trajectory is consists of lines and arcs, trajectory
%   uses arc transition between two lines
%   conti_type means nothing when you only define an arc trajectory
%   TO DO: make the transition radius adaptive
%   Referance: 
%   trajectory planning for automatic machines and robots
%   Author:
%   liao zhixiang, zhixiangleo@163.com

classdef CartesianPlanner < handle

    properties
        segpath_planner
        ntraj
        seg_idx
        t
        cycle_time
        plan_completed

        pos_corner
        rpy_corner
        pos_seg
        rpy_seg
        vmax
        amax
        varc
        trans_radius
        trans_ratio
        continuity
    end

    methods
        function obj = CartesianPlanner(pos_rpy0, conti_type, cycle_time)
            if ~isempty(pos_rpy0)
                obj.pos_corner = pos_rpy0(1:3);
                obj.rpy_corner = pos_rpy0(4:6);
                obj.ntraj = 0;
                obj.continuity = conti_type;
                obj.trans_ratio = 0.3;
                obj.seg_idx = 1; 
                obj.t = 0;
                obj.cycle_time = cycle_time;
                obj.plan_completed = false;
            else
                obj.ntraj = 0;
                obj.continuity = false;
                obj.seg_idx = 1;
                obj.t = 0;
                obj.plan_completed = false;
                obj.cycle_time = cycle_time;
            end
        end
        
        function AddArc(obj,pos1,pos2,pos3,line_vmax,line_amax,rpy1,rpy3,ang_vmax,ang_amax)
            obj.pos_corner = [obj.pos_corner,pos3];
            obj.rpy_corner = [obj.rpy_corner,rpy3];
            obj.ntraj = obj.ntraj+1;                      
            obj.segpath_planner{obj.ntraj} = ArcPlanner(pos1,pos2,pos3,line_vmax,line_amax,[0,0],...
                                                                    rpy1,rpy3,ang_vmax,ang_amax,[0,0], 'arc');
            
        end
        
        %% Add line, arc and their mixture trajectories
        function AddPosRPY(obj,pos_rpy,cvmax,camax)
            obj.vmax = cvmax;
            obj.amax = camax;
            if obj.continuity
                while 1
                    if obj.trans_ratio<0
                        error('can not transition between two lines');
                    end
                    obj.trans_radius = obj.CalcAdaptiveRadius([obj.pos_corner, pos_rpy(1:3,:)]);
                    try
                        for idx=1:size(pos_rpy,2)
                            obj.AddContiPosRPY(pos_rpy(:,idx));
                        end
                        break;
                    catch
                        obj.trans_ratio = obj.trans_ratio-0.05;
                    end
                end
            else
                for idx=1:size(pos_rpy,2)
                    obj.AddDiscontiPosRPY(pos_rpy(:,idx));
                end
            end
        end
        
        function AddContiPosRPY(obj,pos_rpy)
            obj.pos_corner = [obj.pos_corner,pos_rpy(1:3)];
            obj.rpy_corner = [obj.rpy_corner,pos_rpy(4:6)];
            if size(obj.pos_corner,2)==2
                pos0 = obj.pos_corner(:,1);
                posn = obj.pos_corner(:,2);
                rpy0 = obj.rpy_corner(:,1);
                rpyn = obj.rpy_corner(:,2);
                obj.segpath_planner{1} = LinePlanner(pos0,posn,obj.vmax(1),obj.amax(1),[0,0],...
                                                    rpy0,rpyn,obj.vmax(2),obj.amax(2),[0,0]);
                obj.ntraj = 1;
            elseif size(obj.pos_corner,2)==3
                pos_tmp = CalcSplineTransPos(obj.pos_corner(:,1:3),obj.trans_radius,'arc');
                obj.pos_seg = [obj.pos_seg, pos_tmp];
                [~,radius,~] = CalcArcInfo(obj.pos_seg(:,1),obj.pos_corner(:,2),obj.pos_seg(:,2));
                obj.varc(1) = sqrt(obj.amax(1)*radius);
                pos0 = obj.pos_corner(:,1);
                posn = obj.pos_seg(:,1);
                rpy0 = obj.rpy_corner(:,1);
                rpyn = obj.rpy_corner(:,2);
                obj.segpath_planner{1} = LinePlanner(pos0,posn,obj.vmax(1),obj.amax(1),[0,obj.varc(1)],...
                                                    rpy0,rpyn,obj.vmax(2),obj.amax(2),[0,0]);
                obj.ntraj = 3;
            else
                np = size(obj.pos_corner,2);
                pos_tmp = CalcSplineTransPos(obj.pos_corner(:,np-2:np),obj.trans_radius,'arc');
                obj.pos_seg = [obj.pos_seg, pos_tmp];
                [~,radius,~] = CalcArcInfo(obj.pos_seg(:,end-1),obj.pos_corner(:,np-1),obj.pos_seg(:,end));
                obj.varc = [obj.varc,sqrt(obj.amax(1)*radius)];
                obj.ntraj = obj.ntraj+2;
            end
        end
        
        function AddDiscontiPosRPY(obj,pos_rpy)
            obj.pos_corner = [obj.pos_corner, pos_rpy(1:3)];
            obj.rpy_corner = [obj.rpy_corner, pos_rpy(4:6)];
            np = size(obj.pos_corner,2);
            obj.ntraj = obj.ntraj+1;
            pos0 = obj.pos_corner(:,np-1);
            posn = obj.pos_corner(:,np);
            rpy0 = obj.rpy_corner(:,np-1);
            rpyn = obj.rpy_corner(:,np);
            obj.segpath_planner{obj.ntraj} = LinePlanner(pos0,posn,obj.vmax(1),obj.amax(1),[0,0],...
                                                        rpy0,rpyn,obj.vmax(2),obj.amax(2),[0,0]);
        end

        function Stop(obj,cp,cv,cvmax,camax)
            obj.continuity = false;
            obj.plan_completed = false;
            obj.pos_corner = cp(1:3);
            obj.rpy_corner = cp(4:6);
            obj.ntraj = 1;
            obj.seg_idx = 1;
            obj.t = 0;
            obj.segpath_planner = {};
            obj.segpath_planner{1} = LinePlanner(obj.pos_corner,[],cvmax(1),camax(1),[cv(1:3),zeros(3,1)],...
                            obj.rpy_corner,obj.rpy_corner,cvmax(2),camax(2),[0,0]);
            
        end
        %% Generate trajectory
        function [pos,pvel,pacc,rpy,rvel,racc] = GenerateTraj(obj,dt)
            if obj.continuity
                [pos,pvel,pacc,rpy,rvel,racc] = obj.GenerateContiTraj(dt);
            else
                [pos,pvel,pacc,rpy,rvel,racc] = obj.GenerateDiscontiTraj(dt);
            end
        end
        
        function [pos,pvel,pacc,rpy,rvel,racc] = GenerateContiTraj(obj,dt)
            pos = []; pvel = []; pacc = [];
            rpy = []; rvel = []; racc = [];
            for idx=1:obj.ntraj
                [p,vp,ap,r,vr,ar] = obj.segpath_planner{idx}.GenerateTraj(dt);
                % make transition between line and arc smooth
                if idx~=obj.ntraj
                    if mod(idx,2)==1
                        % plan the next trajectory: arc segment
                        pos1 = p(:,end);
                        pos2 = obj.pos_corner(:,(idx+1)/2+1);
                        pos3 = obj.UpdateSegPos(pos1,pos2,obj.pos_corner(:,(idx+1)/2+2));
                        vcons = norm(vp(:,end));
                        obj.segpath_planner{idx+1} = ArcPlanner(pos1,pos2,pos3,vcons,obj.amax(1),[vcons,vcons],...
                                                                    r(:,end),r(:,end),[],[],[], 'arctrans');
                    else
                        % plan the next trajectory: line segment
                        pos0 = p(:,end);
                        rpy0 = r(:,end);
                        rpyn = obj.rpy_corner(:,idx/2+2);
                        v0 = norm(vp(:,end));
                        if idx==obj.ntraj-1
                            posn = obj.pos_corner(:,end);
                            vf = 0;
                        else
                            posn = obj.pos_seg(:,idx+1);
                            vf = obj.varc(idx/2+1);
                        end
                        obj.segpath_planner{idx+1} = LinePlanner(pos0,posn,obj.vmax(1),obj.amax(1),[v0,vf],...
                                            rpy0,rpyn,obj.vmax(2),obj.amax(2),[0,0]);
                    end
                    pos = [pos,p(:,1:end-1)]; pvel = [pvel,vp(:,1:end-1)]; pacc = [pacc,ap(:,1:end-1)];
                    rpy = [rpy,r(:,1:end-1)]; rvel = [rvel,vr(:,1:end-1)]; racc = [racc,ar(:,1:end-1)];
                else
                    pos = [pos,p(:,1:end-1)]; pvel = [pvel,vp(:,1:end-1)]; pacc = [pacc,ap(:,1:end-1)];
                    rpy = [rpy,r(:,1:end-1)]; rvel = [rvel,vr(:,1:end-1)]; racc = [racc,ar(:,1:end-1)];
                end
            end
        end
        
        function [pos,pvel,pacc,rpy,rvel,racc] = GenerateDiscontiTraj(obj,dt)
            pos = []; pvel = []; pacc = [];
            rpy = []; rvel = []; racc = [];
            for idx=1:obj.ntraj
                [p,vp,ap,r,vr,ar] = obj.segpath_planner{idx}.GenerateTraj(dt);
                pos = [pos,p]; pvel = [pvel,vp]; pacc = [pacc, ap];
                rpy = [rpy,r]; rvel = [rvel, vr]; racc = [racc, ar];
            end
        end
        
        %% Generate motion
        function [p,pv,pa,r,rv,ra] = GenerateMotion(obj)
            if obj.continuity
                [p,pv,pa,r,rv,ra] = obj.GenerateContiMotion();
            else
                [p,pv,pa,r,rv,ra] = obj.GenerateDiscontiMotion();
            end
        end
        
        function [p,pv,pa,r,rv,ra] = GenerateContiMotion(obj)
            [p,pv,pa,r,rv,ra] = obj.segpath_planner{obj.seg_idx}.GenerateMotion(obj.t);
            if (obj.seg_idx==obj.ntraj) && (abs(obj.t-obj.segpath_planner{obj.seg_idx}.tf)<1e-5)
                obj.plan_completed = true;
            else
                tf = obj.segpath_planner{obj.seg_idx}.tf;
                tf_int = floor(tf/obj.cycle_time)*obj.cycle_time;
                if (mod(obj.seg_idx,2)==1) && (abs(obj.t-tf_int+obj.cycle_time)<obj.cycle_time) && (obj.seg_idx~=obj.ntraj)
                    % plan the next trajectory: arc segment
                    [pos1,v1,~,rpy1,~,~] = obj.segpath_planner{obj.seg_idx}.GenerateMotion(tf_int);
                    pos2 = obj.pos_corner(:,(obj.seg_idx+1)/2+1);
                    pos3 = obj.UpdateSegPos(pos1,pos2,obj.pos_corner(:,(obj.seg_idx+1)/2+2));
                    vcons = norm(v1(:,end));
                    obj.segpath_planner{obj.seg_idx+1} = ArcPlanner(pos1,pos2,pos3,vcons,obj.amax(1),[vcons,vcons],...
                                                            rpy1,rpy1,[],[],[], 'arctrans');
                    obj.seg_idx = obj.seg_idx+1;
                    obj.t = -obj.cycle_time;
                elseif (mod(obj.seg_idx,2)==0) && (abs(obj.t-tf_int+obj.cycle_time)<obj.cycle_time)
                    % plan the next trajectory: line segment
                    [pos0,vp,~,rpy0,~,~] = obj.segpath_planner{obj.seg_idx}.GenerateMotion(tf_int);
                    rpyn = obj.rpy_corner(:,obj.seg_idx/2+2);
                    v0 = norm(vp);
                    if obj.seg_idx==obj.ntraj-1
                        posn = obj.pos_corner(:,end);
                        vf = 0;
                    else
                        posn = obj.pos_seg(:,obj.seg_idx+1);
                        vf = obj.varc(obj.seg_idx/2+1);
                    end
                    obj.segpath_planner{obj.seg_idx+1} = LinePlanner(pos0,posn,obj.vmax(1),obj.amax(1),[v0,vf],...
                                        rpy0,rpyn,obj.vmax(2),obj.amax(2),[0,0]);
                    obj.seg_idx = obj.seg_idx+1;
                    obj.t = -obj.cycle_time;
                end
                obj.t = obj.t+obj.cycle_time;
                obj.t = LimitNumber(0,obj.t,obj.segpath_planner{obj.seg_idx}.tf);
            end
        end
        
        function [p,pv,pa,r,rv,ra] = GenerateDiscontiMotion(obj)
            [p,pv,pa,r,rv,ra] = obj.segpath_planner{obj.seg_idx}.GenerateMotion(obj.t);
            obj.t = obj.t + obj.cycle_time;
            obj.t = LimitNumber(0,obj.t,obj.segpath_planner{obj.seg_idx}.tf);
            if (obj.seg_idx==obj.ntraj) && (abs(obj.t-obj.segpath_planner{obj.seg_idx}.tf)<obj.cycle_time)
                obj.plan_completed = true;
            else
                if (abs(obj.t-obj.segpath_planner{obj.seg_idx}.tf)<1e-5)
                    obj.seg_idx  = obj.seg_idx + 1;
                    obj.t = 0;
                end
            end
        end
        
        %% mached tool function
        function pos = UpdateSegPos(obj,p1,p2,p3_corner)
            line_len = norm(p2-p1);
            dir_p2p3 = (p3_corner-p2)/norm(p3_corner-p2);
            pos = p2+line_len*dir_p2p3;
        end
        
        function radius = CalcAdaptiveRadius(obj,pos)
            np = size(pos,2);
            for idx=1:np-1
                len(idx) = norm(pos(:,idx+1)-pos(:,idx));
            end
            radius = obj.trans_ratio*min(len)/2;
        end

    end


end



