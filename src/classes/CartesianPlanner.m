classdef CartesianPlanner < handle

    properties
        segpath_planner
        ntraj

        pos_corner
        rpy_corner
        pos_seg
        rpy_seg
        
        varc
        continuity
    end

    methods
        function obj = CartesianPlanner(pos_rpy0, conti_type)
            obj.pos_corner = pos_rpy0(1:3);
            obj.rpy_corner = pos_rpy0(4:6);
            obj.ntraj = 0;
            obj.continuity = conti_type;
        end
        
        function AddPosRPY(obj,pos_rpy)
            if obj.continuity
                for idx=1:size(pos_rpy,2)
                    obj.AddContiPosRPY(pos_rpy(:,idx));
                end
            else
                for idx=1:size(pos_rpy,2)
                    obj.AddDiscontiPosRPY(pos_rpy(:,idx));
                end
            end
        end
        
        function AddContiPosRPY(obj,pos_rpy)
            global g_cvmax g_camax
            obj.pos_corner = [obj.pos_corner,pos_rpy(1:3)];
            obj.rpy_corner = [obj.rpy_corner,pos_rpy(4:6)];
            if size(obj.pos_corner,2)==2
                pos0 = obj.pos_corner(:,1);
                posn = obj.pos_corner(:,2);
                rpy0 = obj.rpy_corner(:,1);
                rpyn = obj.rpy_corner(:,2);
                obj.segpath_planner{1} = LinePlanner(pos0,posn,g_cvmax(1),g_camax(1),[0,0],...
                                                        rpy0,rpyn,g_cvmax(2),g_camax(2),[0,0]);
                obj.ntraj = 1;
            elseif size(obj.pos_corner,2)==3
                pos_tmp = CalcSplineTransPos(obj.pos_corner(:,1:3),0.05,'arc');
                obj.pos_seg = [obj.pos_seg, pos_tmp];
                [~,radius,~] = CalcArcInfo(obj.pos_seg(:,1),obj.pos_corner(:,2),obj.pos_seg(:,2));
                obj.varc(1) = sqrt(g_camax(1)*radius);
                pos0 = obj.pos_corner(:,1);
                posn = obj.pos_seg(:,1);
                rpy0 = obj.rpy_corner(:,1);
                rpyn = obj.rpy_corner(:,2);
                obj.segpath_planner{1} = LinePlanner(pos0,posn,g_cvmax(1),g_camax(1),[0,obj.varc(1)],...
                                                                rpy0,rpyn,g_cvmax(2),g_camax(2),[0,0]);
                obj.ntraj = 3;
            else
                np = size(obj.pos_corner,2);
                pos_tmp = CalcSplineTransPos(obj.pos_corner(:,np-2:np),0.05,'arc');
                obj.pos_seg = [obj.pos_seg, pos_tmp];
                [~,radius,~] = CalcArcInfo(obj.pos_seg(:,end-1),obj.pos_corner(:,np-1),obj.pos_seg(:,end));
                obj.varc = [obj.varc,sqrt(g_camax(1)*radius)];
                obj.ntraj = obj.ntraj+2;
            end
        end
        
        function AddDiscontiPosRPY(obj,pos_rpy)
            global g_cvmax g_camax
            obj.pos_corner = [obj.pos_corner, pos_rpy(1:3)];
            obj.rpy_corner = [obj.rpy_corner, pos_rpy(4:6)];
            np = size(obj.pos_corner,2);
            obj.ntraj = obj.ntraj+1;
            pos0 = obj.pos_corner(:,np-1);
            posn = obj.pos_corner(:,np);
            rpy0 = obj.rpy_corner(:,np-1);
            rpyn = obj.rpy_corner(:,np);
            obj.segpath_planner{obj.ntraj} = LinePlanner(pos0,posn,g_cvmax(1),g_camax(1),[0,0],...
                                                                                        rpy0,rpyn,g_cvmax(2),g_camax(2),[0,0]);
        end

        function [pos,pvel,pacc,rpy,rvel,racc] = GenerateTraj(obj,dt)
            if obj.continuity
                [pos,pvel,pacc,rpy,rvel,racc] = obj.GenerateContiTraj(dt);
            else
                [pos,pvel,pacc,rpy,rvel,racc] = obj.GenerateDiscontiTraj(dt);
            end
        end
        
        function [pos,pvel,pacc,rpy,rvel,racc] = GenerateContiTraj(obj,dt)
            global g_cvmax g_camax
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
                        obj.segpath_planner{idx+1} = ArcPlanner(pos1,pos2,pos3,vcons,g_camax(1),[vcons,vcons],...
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
                        obj.segpath_planner{idx+1} = LinePlanner(pos0,posn,g_cvmax(1),g_camax(1),[v0,vf],...
                                            rpy0,rpyn,g_cvmax(2),g_camax(2),[0,0]);
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
        
        function pos = UpdateSegPos(obj,p1,p2,p3_corner)
            line_len = norm(p2-p1);
            dir_p2p3 = (p3_corner-p2)/norm(p3_corner-p2);
            pos = p2+line_len*dir_p2p3;
        end
        

    end


end