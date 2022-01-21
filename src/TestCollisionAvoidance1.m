clear all
close all
clc

addpath('classes');
addpath(genpath('tools'));

g_jvmax = [pi/8, pi/8, 0.8*pi/4, pi/4, pi/4, pi/4];
g_jamax = [pi/4, pi/4, pi/2, pi/4, pi/4, pi/4];
g_cvmax = [0.15, 0.15]; g_camax = [0.3, 0.3];
g_cycle_time = 0.005;

rbt = CreateRobot();
rbt.tool = SE3(rotx(0), [0,0,0.4]);
dt = 0.01;
q0 = deg2rad([0,-35,20,65,-90,0]');
% q0 = zeros(6,1);
% table zone
p1 = [0.379,0.018,-0.164]'; p2 = [0.407,0.5057,-0.158]';
p3 = [0.852,0.4808,-0.1438]'; p4 = [0.8264,-0.0066,-0.148375]';
line_planner{1} = LinePlanner(rbt.fkine(q0).t,p1,g_cvmax(1),g_camax(1),[0,0],zeros(3,1),zeros(3,1),1,1,[0,0]);
line_planner{2} = LinePlanner(p1,p2,g_cvmax(1),g_camax(1),[0,0],zeros(3,1),zeros(3,1),1,1,[0,0]);
% line_planner{3} = LinePlanner(p2,p3,g_cvmax(1),g_camax(1),[0,0],zeros(3,1),zeros(3,1),1,1,[0,0]);
% line_planner{4} = LinePlanner(p3,p4,g_cvmax(1),g_camax(1),[0,0],zeros(3,1),zeros(3,1),1,1,[0,0]);
% line_planner{5} = LinePlanner(p4,p1,g_cvmax(1),g_camax(1),[0,0],zeros(3,1),zeros(3,1),1,1,[0,0]);
cpos = []; cvel = [];
for idx=1:length(line_planner)
    [p,v,~,~,~,~] = line_planner{idx}.GenerateTraj(dt);
    cpos = [cpos,p]; cvel = [cvel,v];
end

% rbtpoints = CreateRobotPoints(rbt,q0);
% PlotRobotPoints(rbtpoints);

q = q0; qd = zeros(6,1);
jpos = []; jvel = [];
cpos_sim = []; distance = [];
kh_save = []; vo_save = [];

for nidx=1:size(cpos,2)
    q = q+qd*dt;
    cpos_sim = [cpos_sim,rbt.fkine(q).t];
    je = rbt.jacob0(q,'trans');
    je_pinv = pinv(je);
    N = eye(6)-je_pinv*je;
    rbtpoints = CreateRobotPoints(rbt,q);
    xo = 0.5*(rbtpoints{11}+rbtpoints{12});
    jo = CalcJacoObstacle(rbt,q,xo,11);
    jo = jo(1:2,:);
    clean_center = 0.5*(0.5*(p1+p3)+0.5*(p2+p4));
    dis = sqrt((xo(1)-clean_center(1))^2+(xo(2)-clean_center(2))^2);
    dis_soi = 0.3; dis_ni = 0.4; dis_ug = 0.08;
    nvo = clean_center-xo;
    nvo = nvo(1:2);
    nkv = 0.8;
    if dis>dis_ni
        kv = 0;
    elseif (dis<=dis_ni) && (dis>=dis_soi)
        kv = nkv*(dis-dis_ni)/(dis_soi-dis_ni);
    elseif (dis<dis_soi) && (dis>dis_ug)
        kv= nkv;
    else
        kv= nkv*dis/dis_ug;
    end
    vo = kv*nvo;
    
    nkh = 1;
    if dis>dis_ni
        kh = 0;
    elseif (dis<=dis_ni) && (dis>dis_soi)
        kh = 0.5*nkh*(1+cos(pi*((dis-dis_soi)/(dis_ni-dis_soi))));
    elseif (dis<=dis_soi) && (dis>dis_ug)
        kh = nkh;
    else
        kh = 0.5*nkh*(1+sin(pi/2*(2/dis_ug*dis-1)));
    end
    
    Ke = diag([10,10,10]);
    xerr = Ke*(cpos(:,nidx)-rbt.fkine(q).t);
    % original method
%     qd = je_pinv*(cvel(:,nidx)+xerr);
    % method1
    qd = je_pinv*(cvel(:,nidx)+xerr)+kh*pinv(jo*N)*(vo-jo*je_pinv*cvel(:,nidx));
    
    if max(qd)>max(g_jvmax)
        a = 1;
    end

    qd = LimitNumber(-g_jvmax,qd,g_jvmax);
    jpos = [jpos,q]; jvel = [jvel,qd];
    distance = [distance,dis];
    kh_save = [kh_save, kh]; vo_save = [vo_save,vo];
end
t = 0:dt:dt*(size(jpos,2)-1);

figure
plot2([p1,p2,p3,p4,p1]','r--'); hold on; axis equal;
plot2([p1,p2,p3,p4,p1]','r*');
plot2(cpos','k-'); plot2(cpos_sim','g');
grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

figure
plot(distance);
%% robot description
function rbt = CreateRobot()
    d1 = 0.048; a2 = 0.51; a3 = 0.51;
    d4 = 0.11; d5 = 0.08662; d6 = 0.035;
    mdh_table = [0, d1, 0, 0, 0, 0;...
                        0, 0, 0, -pi/2, 0, -pi/2;...
                        0, 0, a2, 0, 0, pi/2;...
                        0, d4, a3, 0, 0, -pi/2;...
                        0, d5, 0, -pi/2, 0, 0;...
                        0, d6, 0, pi/2, 0, 0];
    % pose_tool = SE3(rotx(-10), [0,0,0.116]);
    tool_toiletlid = SE3(rotx(0), [0,-0.035,0.23]);
    qmin = [-pi, -pi/2, -4*pi/3, -pi, -pi, -2*pi]';
    qmax = [pi, pi/2, pi/3, pi, pi, 2*pi]';
    rbt = SerialLink(mdh_table, 'modified', 'name', 'CleanRobot', 'tool',tool_toiletlid);
    rbt.qlim(:,1) = qmin; rbt.qlim(:,2) = qmax;
end

%% robot link description
function rbtpoint = CreateRobotPoints(rbt,q0)
    % baselink1.p1 = [0,0,-0.06]'; baselink1.p2 = [0,0,0.06]';
    % baselink2_1.p1 = [0,0,0]'; baselink2_1.p2 = [0,0,0.17]';
    % baselink2_2.p1 = [0,0,0.11]'; baselink2_2.p2 = [0.51,0,0.11]';
    % baselink3_1.p1 = [0,0,0.17]'; baselink3_1.p2 = [0,0,-0.05]';
    % baselink3_2.p1 = [0,0,0]'; baselink3_2.p2 = [0.51,0,0]';
    % baselink4.p1 = [0,0,-0.16]'; baselink4.p2 = [0,0,0]';
    % baselink5.p1 = [0,0,-0.14]'; baselink5.p2 = [0,0,0]';
    % baselink6.p1 = [0,0,-0.05]'; baselink6.p2 = [0,0,0.43]';
    basepoint{1} = [0,0,-0.06]'; basepoint{2} = [0,0,0.06]';
    basepoint{3} = [0,0,0]'; basepoint{4} = [0,0,0.17]';
    basepoint{5} = [0,0,0.11]'; basepoint{6} = [0.51,0,0.11]';
    basepoint{7} = [0,0,0.17]'; basepoint{8} = [0,0,-0.05]';
    basepoint{9} = [0,0,0]'; basepoint{10} = [0.51,0,0]';
    basepoint{11} = [0,0,-0.16]'; basepoint{12} = [0,0,0]';
    basepoint{13} = [0,0,-0.14]'; basepoint{14} = [0,0,0]';
    basepoint{15} = [0,0,-0.05]'; basepoint{16} = [0,0,0.4]';
    for pidx=1:length(basepoint)
        if pidx>=1 &&pidx<=2
            rot_idx = 1;
        elseif pidx>=3 && pidx<=6
            rot_idx = 2;
        elseif pidx>=7 && pidx<=10
            rot_idx = 3;
        else
            rot_idx = ceil((pidx-4)/2);
        end
        rot = rbt.A(1:rot_idx,q0).R;
        origin = rbt.A(1:rot_idx,q0).t;
        rbtpoint{pidx} = origin+rot*basepoint{pidx};
%         link{pidx}.p1 = origin+rot*baselink{pidx}.p1;
%         link{pidx}.p2 = origin+rot*baselink{pidx}.p2;
    end
    % link1.p1 = rbt.A(1,q0).t+rbt.A(1,q0).R*baselink1.p1;
    % link1.p2 = rbt.A(1,q0).t+rbt.A(1,q0).R*baselink1.p2;
    % link2_1.p1 = rbt.A(1:2,q0).t+rbt.A(1:2,q0).R*baselink2_1.p1;
    % link2_1.p2 = rbt.A(1:2,q0).t+rbt.A(1:2,q0).R*baselink2_1.p2;
    % link2_2.p1 = rbt.A(1:2,q0).t+rbt.A(1:2,q0).R*baselink2_2.p1;
    % link2_2.p2 = rbt.A(1:2,q0).t+rbt.A(1:2,q0).R*baselink2_2.p2;
    % link3_1.p1 = rbt.A(1:3,q0).t+rbt.A(1:3,q0).R*baselink3_1.p1;
    % link3_1.p2 = rbt.A(1:3,q0).t+rbt.A(1:3,q0).R*baselink3_1.p2;
    % link3_2.p1 = rbt.A(1:3,q0).t+rbt.A(1:3,q0).R*baselink3_2.p1;
    % link3_2.p2 = rbt.A(1:3,q0).t+rbt.A(1:3,q0).R*baselink3_2.p2;
    % link4.p1 = rbt.A(1:4,q0).t+rbt.A(1:4,q0).R*baselink4.p1;
    % link4.p2 = rbt.A(1:4,q0).t+rbt.A(1:4,q0).R*baselink4.p2;
    % link5.p1 = rbt.A(1:5,q0).t+rbt.A(1:5,q0).R*baselink5.p1;
    % link5.p2 = rbt.A(1:5,q0).t+rbt.A(1:5,q0).R*baselink5.p2;
    % link6.p1 = rbt.A(1:6,q0).t+rbt.A(1:6,q0).R*baselink6.p1;
    % link6.p2 = rbt.A(1:6,q0).t+rbt.A(1:6,q0).R*baselink6.p2;
end

%% plot robot links
function PlotRobotPoints(rbtpoints)
    figure
    for idx=1:length(rbtpoints)/2
        if idx==1
            plot2([rbtpoints{2*idx-1},rbtpoints{2*idx}]','k-','LineWidth',4);
            hold on; axis equal;
        else
            plot2([rbtpoints{2*idx-1},rbtpoints{2*idx}]','k-','LineWidth',4);
        end
    end
    grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)'); hold off;
end

%% calculate obstacle avoidance point Jacobian
function jo = CalcJacoObstacle(rbt,q,xo,point_idx)
    z0 = [0,0,1]';
    if point_idx>=1 && point_idx<=2
        rot_idx = 1;
    elseif point_idx>=3 && point_idx<=6
        rot_idx = 2;
    elseif point_idx>=7 && point_idx<=10
        rot_idx = 3;
    elseif point_idx>=11 && point_idx<=12
        rot_idx = 4;
    elseif point_idx>=13 && point_idx<=14
        rot_idx = 5;
    elseif point_idx>=15 && point_idx<=16
        rot_idx = 6;
    end
    
    jv = zeros(3,6); jw = zeros(3,6);
    for idx=1:rot_idx
        tmp_pose = rbt.A(1:idx,q);
        rot_0_t = tmp_pose.R;
        pos_0_t = tmp_pose.t;
        if rbt.links(idx).isrevolute()
            jw(:,idx) = rot_0_t*z0;
            jv(:,idx) = cross(jw(:,idx),xo-pos_0_t);
        else
            jw(:,idx) = zeros(3,1);
            jv(:,idx) = rot_0_t*z0;
        end
    end
    
    jo = jv;

end


