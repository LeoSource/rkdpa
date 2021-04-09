classdef DoubleSVelTrajPlanner < handle
    % trajectory with double S velocity profile    
    % assume that: jmin = -jmax, amin = -amax, vmin = -vmax, t0 = 0
    % initial and final accelerations set to zeros
    % generic initial and final values of velocity
    % ref: trajectory planning for automatic machines and robots, chapter 3.4
    
    properties
        q0_hat
        q1_hat
        v0_hat
        v1_hat
        q0
        q1
        v0
        v1
        dir

        vmax
        amax
        jmax
        vmin
        amin
        jmin

        tj1
        ta
        tv
        tj2
        td
        tf

        alima
        alimd
        vlim

        maxvel_reached
        maxacc_reached
    end
      
    methods
        function obj = DoubleSVelTrajPlanner(pos, vel, max_vel, max_acc, max_jerk)
            obj.q0_hat = pos(1); obj.q1_hat = pos(2);
            obj.v0_hat = vel(1); obj.v1_hat = vel(2);
            obj.dir = sign(pos(2)-pos(1));
            obj.TransformPVAJ(max_vel, max_acc, max_jerk);
            if ~obj.ExistTraj()
                error('the displacement is too short');
            else
                obj.maxvel_reached = obj.ReachMaxVel();
            end
            obj.vmin = -obj.vmax; obj.amin = -obj.amax; obj.jmin = -obj.jmax;
            obj.alima = obj.jmax*obj.tj1;
            obj.alimd = -obj.jmax*obj.tj2;
            if obj.maxvel_reached
                obj.vlim = obj.vmax;
            else
                obj.vlim = obj.v0+(obj.ta-obj.tj1)*obj.alima;
            end
            obj.tf = obj.ta+obj.tv+obj.td;
        end
        
        function TransformPVAJ(obj, max_vel, max_acc, max_jerk)
            obj.q0 = obj.dir*obj.q0_hat;
            obj.q1 = obj.dir*obj.q1_hat;
            obj.v0 = obj.dir*obj.v0_hat;
            obj.v1 = obj.dir*obj.v1_hat;
            obj.vmax = 0.5*(obj.dir+1)*max_vel-0.5*(obj.dir-1)*max_vel;
            obj.vmin = -max_vel;
            obj.amax = 0.5*(obj.dir+1)*max_acc-0.5*(obj.dir-1)*max_acc;
            obj.amin = -max_acc;
            obj.jmax = 0.5*(obj.dir+1)*max_jerk-0.5*(obj.dir-1)*max_jerk;
            obj.jmin = -max_jerk;
        end

        function traj_feasible = ExistTraj(obj)
            t1 = sqrt(abs(obj.v1-obj.v0)/obj.jmax);
            t2 = obj.amax/obj.jmax;
            tjmin = min(t1, t2);
            if tjmin<t2
                dis = tjmin*(obj.v0+obj.v1);
            else
                dis = 0.5*(obj.v0+obj.v1)*(tjmin+abs(obj.v1-obj.v0)/obj.amax);
            end
            if abs(obj.q1-obj.q0)>dis
                traj_feasible = 1;
            else
                traj_feasible = 0;
            end
        end

        function maxvel_reached = ReachMaxVel(obj)
            if (obj.vmax-obj.v0)*obj.jmax<obj.amax^2
                obj.tj1 = sqrt((obj.vmax-obj.v0)/obj.jmax);
                obj.ta = 2*obj.tj1;
            else
                obj.tj1 = obj.amax/obj.jmax;
                obj.ta = obj.tj1+(obj.vmax-obj.v0)/obj.amax;
            end
            if (obj.vmax-obj.v1)*obj.jmax<obj.amax^2
                obj.tj2 = sqrt((obj.vmax-obj.v1)/obj.jmax);
                obj.td = 2*obj.tj2;
            else
                obj.tj2 = obj.amax/obj.jmax;
                obj.td = obj.tj2+(obj.vmax-obj.v1)/obj.amax;
            end
            obj.tv = (obj.q1-obj.q0)/obj.vmax-0.5*obj.ta*(1+obj.v0/obj.vmax)...
                    -0.5*obj.td*(1+obj.v1/obj.vmax);
            if obj.tv<0
                % the constant velocity segment is not present
                % vlim < vmax
                maxvel_reached = 0;
                obj.tv = 0;
                obj.maxacc_reached = obj.ReachMaxAcc();
            else
                % there is constant velocity segment
                % vlim = vmax
                maxvel_reached = 1;
            end
        end

        function maxacc_reached = ReachMaxAcc(obj)
            [obj.ta, obj.td, obj.tj1, obj.tj2] = deal(0);
            gam = 1; step = -0.005;
            % recursive algorithm for that maximum acceleration is not reached
            while ~(obj.ta>2*obj.tj1 && obj.td>2*obj.tj2) && (obj.ta>=0) && (obj.td>=0)
                obj.amax = gam*obj.amax;
                obj.tj1 = obj.amax/obj.jmax;
                obj.tj2 = obj.tj1;
                delta = obj.amax^4/obj.jmax^2+2*(obj.v0^2+obj.v1^2)...
                        +obj.amax*(4*(obj.q1-obj.q0)-2*obj.amax/obj.jmax*(obj.v0+obj.v1));
                obj.ta = (obj.amax^2/obj.jmax-2*obj.v0+sqrt(delta))/(2*obj.amax);
                obj.td = (obj.amax^2/obj.jmax-2*obj.v1+sqrt(delta))/(2*obj.amax);
                gam = gam+step;
            end
            % the acceleration phase is not present
            if obj.ta<0
                obj.tj1 = 0; obj.ta = 0;
                obj.td = 2*(obj.q1-obj.q0)/(obj.v0+obj.v1);
                tmp_value = sqrt(obj.jmax*(obj.jmax*(obj.q1-obj.q0)^2+(obj.v0+obj.v1)^2*(obj.v1-obj.v0)));
                tmp_value = obj.jmax*(obj.q1-obj.q0)-tmp_value;
                obj.tj2 = tmp_value/obj.jmax/(obj.v0+obj.v1);
            end
            % the deceleration phase is not present
            if obj.td<0
                obj.tj2 = 0; obj.td = 0;
                obj.ta = 2*(obj.q1-obj.q0)/(obj.v0+obj.v1);
                tmp_value = sqrt(obj.jmax*(obj.jmax*(obj.q1-obj.q0)^2-(obj.v0+obj.v1)^2*(obj.v1-obj.v0)));
                tmp_value = obj.jmax*(obj.q1-obj.q0)-tmp_value;
                obj.tj1 = tmp_value/obj.jmax/(obj.v0+obj.v1);
            end
            if gam<1
                maxacc_reached = 0;
            else
                maxacc_reached = 1;
            end
        end

      
        function [pos, vel, acc, jerk] = GenerateTraj(obj, dt)
            pos = []; vel = []; acc = []; jerk = [];
            for t = 0:dt:obj.tf
                [p, v, a, j] = obj.GenerateMotion(t);
                pos = [pos, p];
                vel = [vel, v];
                acc = [acc, a];
                jerk = [jerk, j];
            end            
        end       

        function [p, v, a, j] = GenerateMotion(obj, t)
            % acceleration phase %
            if t>=0 && t<=obj.tj1
                p = obj.q0+obj.v0*t+obj.jmax*t^3/6;
                v = obj.v0+0.5*obj.jmax*t^2;
                a = obj.jmax*t;
                j = obj.jmax;
            elseif t>obj.tj1 && t<=(obj.ta-obj.tj1)
                p = obj.q0+obj.v0*t+obj.alima/6*(3*t^2-3*obj.tj1*t+obj.tj1^2);
                v = obj.v0+obj.alima*(t-0.5*obj.tj1);
                a = obj.alima;
                j = 0;
            elseif t>(obj.ta-obj.tj1) && t<=obj.ta
                p = obj.q0+(obj.vlim+obj.v0)*obj.ta*0.5...
                    -obj.vlim*(obj.ta-t)-obj.jmin*(obj.ta-t)^3/6;
                v = obj.vlim+obj.jmin*0.5*(obj.ta-t)^2;
                a = -obj.jmin*(obj.ta-t);
                j = obj.jmin;
            % constant velocity phase
            elseif t>obj.ta && t<=obj.ta+obj.tv
                p = obj.q0+(obj.vlim+obj.v0)*obj.ta*0.5+obj.vlim*(t-obj.ta);
                v = obj.vlim;
                a = 0;
                j = 0;
            % deceleration phase %
            elseif t>obj.tf-obj.td && t<=obj.tf-obj.td+obj.tj2
                p = obj.q1-(obj.vlim+obj.v1)*obj.td*0.5...
                    +obj.vlim*(t-obj.tf+obj.td)-obj.jmax*(t-obj.tf+obj.td)^3/6;
                v = obj.vlim-obj.jmax*(t-obj.tf+obj.td)^2*0.5;
                a = -obj.jmax*(t-obj.tf+obj.td);
                j = obj.jmin;
            elseif t>obj.tf-obj.td+obj.tj2 && t<obj.tf-obj.tj2
                p = obj.q1-(obj.vlim+obj.v1)*obj.td*0.5+obj.vlim*(t-obj.tf+obj.td)...
                    +obj.alimd/6*(3*(t-obj.tf+obj.td)^2-3*obj.tj2*(t-obj.tf+obj.td)+obj.tj2^2);
                v = obj.vlim+obj.alimd*(t-obj.tf+obj.td-0.5*obj.tj2);
                a = obj.alimd;
                j = 0;
            elseif t>obj.tf-obj.tj2 && t<=obj.tf
                p = obj.q1-obj.v1*(obj.tf-t)-obj.jmax*(obj.tf-t)^3/6;
                v = obj.v1+obj.jmax*(obj.tf-t)^2*0.5;
                a = -obj.jmax*(obj.tf-t);
                j = obj.jmax;
            end
            % transform
            p = obj.dir*p; v = obj.dir*v; a = obj.dir*a; j = obj.dir*j;
        end
        
        function PlotMotion(obj, dt, option)
            [q, qd, qdd, qddd] = obj.GenerateTraj(dt);
            motion_data = [q; qd; qdd; qddd];
            motion_name = {'position', 'velocity', 'acceleration', 'jerk'};
            t = 0:dt:obj.tf;
            switch option
            case 'p'
                nplot = 1;
            case 'pv'
                nplot = 2;
            case 'pva'
                nplot = 3;
            case 'pvaj'
                nplot = 4;
            otherwise
                error('input the right motion data plot option');
            end
            for idx=1:nplot
                subplot(nplot, 1, idx)
                plot(t, motion_data(idx,:)); grid on; 
                xlabel('time'); ylabel(motion_name{idx});
            end
        end
        
        
    end
end

