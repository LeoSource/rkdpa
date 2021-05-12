clear
close all
clc

addpath('classes');
addpath('tools');

rbt = CleanRobot;
global g_jvmax g_jamax g_cvmax g_camax g_stowed_pos
g_jvmax = [pi, 0.15, 0.8*pi, 0.5, 0.8*pi];
g_jamax = [2*pi, 0.3, 1.6*pi, 1, 1.6*pi];
g_cvmax = 0.4; g_camax = 0.8;
g_stowed_pos = [0;0;0;0;0];
g_cycle_time = 0.001;
test_mode = 'mirrortask';
switch test_mode
    case 'dhmodel'
%% validation for robot model by simscape
tf = 10;
t = [0:g_cycle_time:tf]';
for idx=1:rbt.nlinks-1
    q0 = rand; a = rand; b = rand;
    q(:, idx) = q0+a*sin(t)+b*sin(2*t);
    qd(:, idx) = a*cos(t)+2*b*cos(2*t);
    qdd(:, idx) = -a*sin(t)-4*b*sin(2*t);
end
cart_pos = [];
for idx=1:size(t,1)
    q_in = [q(idx, 1:3), 0, q(idx, 4)];
    tmp_frame = rbt.FKSolve(q_in);
    end_rot = [tmp_frame.n, tmp_frame.o, tmp_frame.a];
    cart_pos = [cart_pos, tmp_frame.t];
end
sim('simulink/test_model');
sim_pos(:,1) = sim_pos_x; sim_pos(:,2) = sim_pos_y; sim_pos(:,3) = sim_pos_z;
%figure
figure
subplot(2,1,1); plot(t,sim_pos_x ,t, cart_pos(1,:),'--'); grid on;
xlabel('time(s)'); ylabel('pos_x(m)'); legend('simscape model', 'mdh model');
subplot(2,1,2); plot(t, sim_pos_x - cart_pos(1,:)'); legend('position error');
figure
subplot(2,1,1); plot(t,sim_pos_y ,t, cart_pos(2,:),'--'); grid on;
xlabel('time(s)'); ylabel('pos_y(m)'); legend('simscape model', 'mdh model');
subplot(2,1,2); plot(t, sim_pos_y - cart_pos(2,:)'); legend('position error');
figure
subplot(2,1,1); plot(t,sim_pos_z ,t, cart_pos(3,:),'--'); grid on;
xlabel('time(s)'); ylabel('pos_z(m)'); legend('simscape model', 'mdh model');
subplot(2,1,2); plot(t, sim_pos_z - cart_pos(3,:)');legend('position error');

    case  'workspace'
%% validation for the workspace of cleanrobot    
rbt.PlotWorkspace;

    case 'jtrajpoly'
%% joint trajectory plan using polynomial trajectory
q = [0, 0.5, 0.8, 1.1, 1.6, 1.9, 2.2, 2.5, 3];
t = [0, 0.5, 0.8, 1.1, 1.6, 1.9, 2.2, 2.5, 3];
dt = 0.005;
% t = 18;
% t = 8;
planner1 = PolyTrajPlanner(q(1:3), t(1:3), [0,1], 3);
planner2 = PolyTrajPlanner(q(end-2:end),t(end-2:end),[1,0],3);
pos = []; vel = []; acc = [];
for tt=t(1):dt:t(end)
    if tt>=t(1) && tt<=t(3)
        [p, v, a] = planner1.GenerateMotion(tt);
    elseif tt>=t(end-2) && tt<=t(end)
        [p, v, a] = planner2.GenerateMotion(tt);
    else
        p = tt; v = 1; a = 0;
    end
    pos = [pos,p]; vel = [vel,v]; acc=[acc,a];
end
time=t(1):dt:t(end);
figure
subplot(3,1,1); plot(time,pos,'k-'); grid on; ylabel('u(t)');
subplot(3,1,2); plot(time,vel,'k-'); grid on; ylabel('du(t)');
subplot(3,1,3); plot(time,acc,'k-'); grid on; ylabel('ddu(t)');

planner1 = CubicSplinePlanner(q, t, 'clamped', [0, 0]);
% planner1.SetTimeOptimizedConstrtaints(10, 6);
planner1.SetTimeOptimizedStyle('gentle');
% planner1.SetSmoothWeight(0.7);
% planner1.SetSmoothTolerance(0.2);
planner1.PlotAVP(0.001);

    case 'jonlinetraj'
%%  joint online trajectory plan
tf = 25; dt = 0.001; vmax = 3; amax = 2; jmax = 5;
planner = OnlineTrajPlanner(vmax, amax, jmax, dt, 5);
p = 0; v =0; a = 0; j =0;
planner.InitPlanner(p ,v, a);
% generate command motion
pos_cmd = []; vel_cmd = [];
for t=0:dt:tf
    if t>=0 && t<1
        v_cmd = 0;
        p_cmd = 0;
    elseif t>=1 && t<7
        v_cmd = 0;
        p_cmd = 4;
    elseif t>=7 && t<13
        v_cmd = 0;
        p_cmd = 10;
    else
        v_cmd = 0;
        p_cmd = -2;
    end
    pos_cmd = [pos_cmd, p_cmd];
    vel_cmd = [vel_cmd, v_cmd];
end
% generate actual motion
pos = []; vel = []; acc = []; jerk = [];
idx = 0;
for t=0:dt:tf
    idx = idx+1;
    p = p+0.001*sign(rand-0.5);
    v = v+0.01*sign(rand-0.5);
    a = a+0.1*sign(rand-0.5);
    [p, v, a, j] = planner.GenerateMotion5th([pos_cmd(idx),vel_cmd(idx),0], [p,v,a]);
    pos = [pos, p]; vel = [vel, v]; acc = [acc, a]; jerk = [jerk, j];
end

tt = 0:dt:tf;
subplot(3,1,1)
plot(tt, pos, 'k-'); grid on; ylabel('position'); hold on;
plot(tt, pos_cmd, 'r--');
subplot(3,1,2)
plot(tt, vel, 'k-'); grid on; ylabel('velocity'); hold on;
plot(tt, vel_cmd, 'r--');
subplot(3,1,3)
plot(0:dt:tf, acc, 'k-'); grid on; ylabel('acceleration');

    case 'jtrajlspb'
%% joint trajectory plan using lspb 
planner = LspbTrajPlanner([0,0], 2, 1);
planner.PlotAVP(0.01);
planner = LspbTrajPlanner([20, 10], 16, 10, [0,3]);
[p, v, a] = planner.GenerateMotion(1)

trajplanner = DoubleSVelTrajPlanner([10, 0],[0, 0], 5, 10, 30);
trajplanner.SetPhaseDuration(5, 1/3, 1/5);
trajplanner.PlotMotion(0.001, 'pvaj');

% [p, v, a] = planner.GenerateMotion(2)
    case 'ctrajline'
%% cartesian line trajectory plan using lspb
pos_initial = [0, 0, 0];
pos_goal = [3, 4, 5];
line_length = norm(pos_goal-pos_initial);
planner = LspbTrajPlanner([0, line_length], 2, 5, 5);
[p, v, ~] = planner.GenerateTraj(0.01);
pos = pos_initial+p'.*(pos_goal-pos_initial)/line_length;
vel = v'.*(pos_goal-pos_initial)/line_length;
figure
plot2(pos); grid on; hold on;
scatter3(pos_initial(1), pos_initial(2), pos_initial(3));
scatter3(pos_goal(1), pos_goal(2), pos_goal(3));
xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

figure
time = 0:0.01:5;
subplot(3,1,1); plot(time, pos(:,1)); ylabel('px');
subplot(3,1,2); plot(time, pos(:,2)); ylabel('py');
subplot(3,1,3); plot(time, pos(:,3)); ylabel('pz');
figure
subplot(3,1,1); plot(time, vel(:,1)); ylabel('vx');
subplot(3,1,2); plot(time, vel(:,2)); ylabel('vy');
subplot(4,1,3); plot(time, vel(:,3)); ylabel('vz');

    case 'ctrajarc'
%% cartesian arc path trajectory plan using lspb
pos1 = [0, -2, 0];
pos2 = [1, 0, 1];
pos3 = [0, 3, 3];
arcpath = ArcPathPlanner(pos1, pos2, pos3, 'arc');
planner = LspbTrajPlanner([0, arcpath.theta], 2, 2, 2, 'limitvel');
[alp, alpv, alpa] = planner.GenerateTraj(0.01);
[pos, vel, acc] = arcpath.GenerateTraj(alp, alpv, alpa);

f1 = plot3([pos1(1); pos2(1); pos3(1)], [pos1(2); pos2(2); pos3(2)], [pos1(3); pos2(3); pos3(3)], 'ro');
grid on
xlabel('x'); ylabel('y'); zlabel('z');
hold on
f2 = plot3(pos(1,:), pos(2,:), pos(3,:), 'b--');
% comparison test with Cpp
pos_cpp = load('./data/ctrajarc_pos.csv');
np = length(pos_cpp)/3;
pos_cpp = reshape(pos_cpp, 3, np);
f3 = plot3(pos_cpp(1,:), pos_cpp(2,:), pos_cpp(3,:), 'r-');
legend([f2, f3], 'matlab\_data', 'cpp\_data');
arcpath.PlotTraj(alp, alpv, alpa, 2, 0.01);

    case 'ctrajcircle'
%% cartesian circle path trajectory plan using lspb
% test for spatial helix
center = [0,0.7,1]';
radius = 0.6;
interval = 0.1;
v = interval/2/pi;
theta_end = radius/v;
thplanner = LspbTrajPlanner([0,theta_end], 0.7, 0.2);
[thp, thv, tha] = thplanner.GenerateTraj(0.01);
r = v*thp;
x = center(1)+r.*cos(thp);
y = center(2)*ones(size(thp));
z = center(3)+r.*sin(thp);
dx = -r.*sin(thp).*thv;
dy = zeros(size(thp));
dz = r.*cos(thp).*thv;
ddx = -r.*sin(thp).*tha-r.*cos(thp).*thv.^2;
ddy = zeros(size(thp));
ddz = r.*cos(thp).*tha-r.*sin(thp).*thv.^2;
for idx=1:length(thp)
    v(idx) = sqrt(dx(idx)^2+dy(idx)^2+dz(idx)^2);
    a(idx) = sqrt(ddx(idx)^2+ddy(idx)^2+ddz(idx)^2);
end

t = linspace(0,thplanner.tf,length(thp));
figure
plot3(x,y,z, 'k'); grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
figure
subplot(4,2,1); plot(t,dx, 'k'); ylabel('vx'); grid on;
subplot(4,2,3); plot(t,dy, 'k'); ylabel('vy'); grid on;
subplot(4,2,5); plot(t,dz, 'k'); ylabel('vz'); grid on;
subplot(4,2,7); plot(t,v, 'k'); ylabel('|v|'); grid on;
subplot(4,2,2); plot(t,ddx, 'k'); ylabel('ax'); grid on;
subplot(4,2,4); plot(t,ddy, 'k'); ylabel('ay'); grid on;
subplot(4,2,6); plot(t,ddz, 'k'); ylabel('az'); grid on;
subplot(4,2,8); plot(t,a, 'k'); ylabel('|a|'); grid on;
figure
thplanner.PlotAVP(0.01);

% test for spatial circle
%{
center = [1;2;3]; n_vec = [1;0;0]; radius = 0.5; tf = 4;
circlepath = ArcPathPlanner(center, n_vec, radius, 'circle');
planner = LspbTrajPlanner([0, circlepath.theta], 2, 4, tf);
[alp, alpv, alpa] = planner.GenerateTraj(0.01);
[pos, vel, acc] = circlepath.GenerateTraj(alp, alpv, alpa);
plot3(pos(1,:), pos(2,:), pos(3,:));
grid on
xlabel('x'); ylabel('y'); zlabel('z');
circlepath.PlotTraj(alp, alpv, alpa, tf, 0.01);
%}
% comparison test with Cpp
% [alp, alpv, alpa] = planner.GenerateMotion(2);
% [pos, vel, acc] = circlepath.GenerateMotion(alp, alpv, alpa)

    case 'ctrajarctrans'
%% cartesian trajectory for points using arc to transmit between 2 line paths
pos1 = [0.7, 0.8, 1]; pos2 = [-0.7, 0.8, 1]; pos3 = [-0.7, 0.8, 2.4]; pos4 = [0.7, 0.8, 2.4];
% pos1 = [0.8, 0.2, 0.7]; pos2 = [-0.8, 0.2, 0.7]; pos3 = [-0.8, 0.8, 0.7]; pos4 = [0.8, 0.8, 0.7];
via_pos = CalcRectanglePath([pos1', pos2', pos3', pos4'], 's');
% cpath = ArcTransPathPlanner(via_pos, 0);   
% splanner = LspbTrajPlanner([0,cpath.distance], 0.5 , 2);
% [s, sv, sa] = splanner.GenerateMotion(10);
% [pos, vel, acc] = cpath.GenerateMotion(s, sv, sa)
continuity = 1;
max_vel = 0.4; max_acc = 0.8;
if continuity
    dt = 0.01;
    via_pos = CalcRectanglePath([pos1', pos2', pos3', pos4'], 's');
    cpath = ArcTransPathPlanner(via_pos, 0);    
    varc = sqrt(max_acc*cpath.radius);
    s= []; sv = []; sa = [];
    for idx=1:length(cpath.dis_interval)-1
        if mod(idx,2)==1
            if idx==1
                vel_cons = [0, varc];
            elseif idx==length(cpath.dis_interval)-1
                vel_cons = [varc, 0];
            else
                vel_cons = [varc, varc];
            end
            jplanner = LspbTrajPlanner([cpath.dis_interval(idx), cpath.dis_interval(idx+1)],max_vel,max_acc,[],vel_cons);
            [s_tmp, sv_tmp, sa_tmp] = jplanner.GenerateTraj(dt);
        else
            t_len = (cpath.dis_interval(idx+1)-cpath.dis_interval(idx))/varc;
            num_interval = floor(t_len/dt+1);
            sa_tmp = zeros(1, num_interval);
            sv_tmp = ones(1, num_interval)*varc;
            s_tmp = linspace(cpath.dis_interval(idx), cpath.dis_interval(idx+1), num_interval);
        end
        s = [s, s_tmp]; sv = [sv, sv_tmp]; sa = [sa, sa_tmp];
    end   
    [pos, vel, acc] = cpath.GenerateTraj(s, sv, sa);
    tf = dt*(length(s)-1);
    
    figure
    plot3(via_pos(1,:), via_pos(2,:), via_pos(3,:), 'ro');
    grid on; xlabel('x'); ylabel('y'); zlabel('z'); hold on;
    plot3(pos(1,:), pos(2,:), pos(3,:), 'b-');
    cpath.PlotTraj(s, sv, sa, tf, dt);
else
    dt = 0.01;
    via_pos = CalcRectanglePath([pos1', pos2', pos3', pos4'], 's');
    pos = []; vel = []; acc = [];
    for idx=1:size(via_pos,2)-1
        dis = norm(via_pos(:,idx+1)-via_pos(:,idx));
        line_dir = (via_pos(:,idx+1)-via_pos(:,idx))/dis;
        jplanner = LspbTrajPlanner([0,dis], max_vel, max_acc);
        [s,sv,sa] = jplanner.GenerateTraj(dt);
        p = via_pos(:,idx)+s.*line_dir;
        v = sv.*line_dir;
        a = sa.*line_dir;
        pos = [pos, p]; vel = [vel, v]; acc = [acc, a];
    end
    vel_norm = []; acc_norm = [];
    for idx=1:size(pos,2)
        vel_norm = [vel_norm, norm(vel(:,idx))];
        acc_norm = [acc_norm, norm(acc(:,idx))];
    end
    time = [0:size(pos,2)-1]*dt;
    figure
    plot3(via_pos(1,:), via_pos(2,:), via_pos(3,:), 'ro');
    grid on; xlabel('x'); ylabel('y'); zlabel('z'); hold on;
    plot3(pos(1,:), pos(2,:), pos(3,:), 'b-');
    figure
    subplot(3,1,1); plot(time, pos(1,:), 'k-'); grid on; ylabel('px');
    subplot(3,1,2); plot(time, pos(2,:), 'k-'); grid on; ylabel('py');
    subplot(3,1,3); plot(time, pos(3,:), 'k-'); grid on; ylabel('pz');
    figure
    subplot(4,2,1); plot(time, vel(1,:), 'k-'); grid on; ylabel('vx');
    subplot(4,2,3); plot(time, vel(2,:), 'k-'); grid on; ylabel('vy');
    subplot(4,2,5); plot(time, vel(3,:), 'k-'); grid on; ylabel('vz');
    subplot(4,2,7); plot(time, vel_norm, 'k-'); grid on; ylabel('|v|');
    subplot(4,2,2); plot(time, acc(1,:), 'k-'); grid on; ylabel('ax');
    subplot(4,2,4); plot(time, acc(2,:), 'k-'); grid on; ylabel('ay');
    subplot(4,2,6); plot(time, acc(3,:), 'k-'); grid on; ylabel('az');
    subplot(4,2,8); plot(time, acc_norm, 'k-'); grid on; ylabel('|a|');
end

    case 'ctrajpoly'
%% cartesian trajectory plan using polynomial trajectory
% it is very important to define the time sequence for every via position
pos1 = [0.8, 0.2, 0.7]; pos2 = [-0.77, 0.2, 0.7]; pos3 = [-0.77, 0.8, 0.7]; pos4 = [0.8, 0.8, 0.7];
via_pos = CalcRectanglePath1([pos1', pos2', pos3', pos4'], 0.1, 'm');
for idx=1:3
    planner(idx) = PolyTrajPlanner(via_pos(idx,:), 2, 3);
    [pos(idx,:), ~, ~] = planner(idx).GenerateTraj(0.01);
end
% planner(1).PlotAVP(0.01);
figure
plot3(via_pos(1,:), via_pos(2,:), via_pos(3,:), 'ro');
grid on
xlabel('x'); ylabel('y'); zlabel('z');
hold on
plot3(pos(1,:), pos(2,:), pos(3,:));

    case 'ctrajbspline'
%% cartesian trajectory plan using B-spline
%{
interpolation test with cpp
via_pos = [1, 2, 3, 4, 5; 2, 3, -3, 4, 5; 0, 0, 0, 0, 0];
planner = CubicBSplinePlanner(via_pos, 'interpolation', 5);
pos = []; vel = []; acc = [];
for u=planner.knot_vec(1):0.01:planner.knot_vec(end)
    [p,v,a] = planner.GenerateMotion(u,1,0);
    pos = [pos, p]; vel = [vel, v]; acc = [acc, a];
end
cpp_data = load('./data/test_bspline.csv');
cpp_data = reshape(cpp_data, 3, []);
%}

%{
hemisphere simulation
npts = [15, 10, 8, 6];
center = [0, 0, 0, 0; 0, 0, 0, 0; 0, -0.1, -0.2, -0.3];
radius = [0.5, 0.4, 0.3, 0.2];
via_pos = [];
for idx=1:length(npts)
    theta = linspace(0, 2*pi, npts(idx));
    tmp_pos = center(:,idx)+[radius(idx)*cos(theta); radius(idx)*sin(theta); zeros(1,npts(idx))];
    via_pnts{idx} = tmp_pos;
    via_pos = [via_pos, tmp_pos];
end
%}
npts = [15, 12, 10, 8, 6, 4];
center = [0,0,0,0,0,0; 0,0,0,0,0,0; 0,-0.06,-0.12,-0.18,-0.24,-0.3];
elli_params = [0.7, 0.6, 0.5, 0.4, 0.3, 0.2; 0.5, 0.4, 0.3, 0.2, 0.1, 0.05];
via_pos = [];
for idx=1:length(npts)
    theta = linspace(0,2*pi,npts(idx)+1);
    tmp_pos = center(:,idx)+[elli_params(1,idx)*cos(theta); elli_params(2,idx)*sin(theta); zeros(1,npts(idx)+1)];
    via_pos = [via_pos, tmp_pos];
end
planner = CubicBSplinePlanner(via_pos, 'approximation', 60);
planner.PlotAVP(0.01);
planner.PlotBSpline(0.01); hold on;
if size(via_pos,1)==3
    scatter3(via_pos(1,:), via_pos(2,:), via_pos(3,:));
else
    scatter(via_pos(1,:), via_pos(2,:));
end

    case 'jacobian'
%% test for jacobian
l1 = 0.106; l2 = 0.09; l3 = 0;% model refinement
h = 0.5; w = 0.423;
mdh_table = [      0,   0,   0,       0,    0,   0
                pi/2,   0,   0,       0,    1,   h 
                    0,    l2,   -l1,   pi/2,    0,   pi/2
                pi/2,    0,   0,   pi/2,   1,   w
                    0,    0,   -l3,    pi/2,   0,   0
                    0,   0.2,  0,   -pi/6-pi/2, 0, 0];
test_rbt = SerialLink(mdh_table, 'modified', 'name', 'test_rbt');
q = rand(1,5);  test_q = [q, 0];
dq = rand(5,1); test_dq = [dq; 0];
spatial_vel = rbt.CalcJacoTool(q)*dq
test_jaco = test_rbt.jacob0(test_q);
test_vel = test_jaco*test_dq
jaco_err = norm(rbt.CalcJacoTool(q)-test_jaco(:,1:5))

q= [0.3,0.98,-0.91,0.91,0.56];
jaco = rbt.CalcJacoTool(q)
    case 'redudantsolve'
%% redundant solve for robot work
pos1 = [0.5, 0.8, 1]; pos2 = [-0.5, 0.8, 1]; pos3 = [-0.5, 0.8, 1.8]; pos4 = [0.5, 0.8, 1.8];
% pos1 = [0.8, 0.2, 0.7]; pos2 = [-0.77, 0.2, 0.7]; pos3 = [-0.77, 0.8, 0.7]; pos4 = [0.8, 0.8, 0.7];
radius = 0.04;
tf = 40; dt = 0.001;
via_pos = CalcRectanglePath2([pos1', pos2', pos3', pos4'], 9, 's');
cpath = ArcTransPathPlanner(via_pos, radius);
planner = LspbTrajPlanner([0, cpath.distance], tf, 0.5, 2, 'limitvel');
[s, sv, sa] = planner.GenerateTraj(dt);
[pos, vel, acc] = cpath.GenerateTraj(s, sv, sa);
q = rbt.IKSolve(pos1, 'q2first', 0);
sim_q = []; sim_pos = [];
rbt.InitIKSolver(q, dt);
for idx=1:size(pos, 2)
%     [q, qd] = rbt.IKSolvePos(pos(:,idx), vel(:,idx), q);
%     [q, qd] = rbt.IKSolveRPY([pos(:,idx);0;0], [vel(:,idx);0;0], q);
    [q, qd] = rbt.IKSolveRedudant(pos(:,idx), vel(:,idx), q, [0;1;0]);
    pose = rbt.FKSolveTool(q);
    tmp_pos = pose(1:3, end);
    sim_q = [sim_q, q]; sim_pos = [sim_pos, tmp_pos];
end

t = [0:dt:tf]';
plot(t,sim_q(1, :),'-', t, sim_q(2, :), '--', t, sim_q(3, :), '-.', t, sim_q(4, :), ':', t, sim_q(5, :), '-');
grid on; title('joint position'); legend('q1', 'q2', 'q3', 'q4', 'q5');
figure
plot2(pos', 'r--'); hold on; plot2(sim_pos', 'k-'); grid on; axis([-inf, inf, 0.6, 1, -inf, inf]);
xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)'); legend('cmd\_pos', 'sim\_pos');
    case 'mirrortask'
%%  comparison with cpp
q0 = [0.2,0.8,0,0,0]';
dt = 0.01;
mirror_style = 'circle';
comparison = 0;
if strcmp(mirror_style, 'rectangle')
    pos1 = [0.7, 0.8, 1]; pos2 = [-0.7, 0.8, 1]; pos3 = [-0.7, 0.8, 2.4]; pos4 = [0.7, 0.8, 2.4];
    via_pos = CalcRectanglePath([pos1', pos2', pos3', pos4'], 's');
    [~, sim_q] = CleanRectMirror(rbt, via_pos, q0, dt);
elseif strcmp(mirror_style, 'circle')
    center = [0,0.7,1]';
    radius = 0.6;
    interval = 0.1;
    [~, sim_q] = CleanCircleMirror(rbt, center, radius, interval, q0, dt);
end 

t = 0:dt:dt*(size(sim_q,2)-1);
plot(t,sim_q(1,:),'-', t, sim_q(2,:), '--', t, sim_q(3,:), '-.', t, sim_q(4,:), ':', t, sim_q(5,:), '-');
grid on; title('joint position'); legend('q1', 'q2', 'q3', 'q4', 'q5');
if comparison
    q_cpp = load('./data/mirrortask_jpos1.csv');
    q_cpp = reshape(q_cpp, rbt.nlinks, length(q_cpp)/5);
    tt = g_cycle_time*[0:size(q_cpp,2)-1];
    for idx=1:rbt.nlinks
        figure
        plot(t, sim_q(idx,:), 'b--'); xlabel('time'); ylabel(['q', num2str(idx)]); grid on;
        plot(t, sim_q(idx,:), 'b--', tt, q_cpp(idx,:), 'r-');
        xlabel('time'); ylabel(['q', num2str(idx)]); grid on;
        legend('matlab\_data', 'cpp\_data');
    end
end

    case 'washbasin'
%% comparison with cpp
q0 = [0.2,0.8,0,0,0]';
dt = 0.01;
npts = [15, 10, 8, 6];
center = [0, 0, 0, 0; 0.7, 0.7, 0.7, 0.7; 0.6-0, 0.6-0.1, 0.6-0.2, 0.6-0.3];
radius = [0.4, 0.3, 0.2, 0.1];
via_pos = [];
for idx=1:length(npts)
    theta = linspace(0, 2*pi, npts(idx)+1);
    tmp_pos = center(:,idx)+[radius(idx)*cos(theta); radius(idx)*sin(theta); zeros(1,npts(idx)+1)];
    via_pos = [via_pos, tmp_pos];
end
[sim_pos, sim_q, pos] = CleanSurface(rbt,npts,center(:,1),via_pos,q0,dt);

t = 0:dt:dt*(size(sim_q,2)-1);
plot(t,sim_q(1,:),'-', t, sim_q(2,:), '--', t, sim_q(3,:), '-.', t, sim_q(4,:), ':', t, sim_q(5,:), '-');
grid on; title('joint position'); legend('q1', 'q2', 'q3', 'q4', 'q5');
q_cpp = load('./data/mirrortask_jpos1.csv');
q_cpp = reshape(q_cpp, rbt.nlinks, length(q_cpp)/5);
tt = g_cycle_time*[0:size(q_cpp,2)-1];
for idx=1:rbt.nlinks
    figure
    plot(t, sim_q(idx,:), 'b--'); xlabel('time'); ylabel(['q', num2str(idx)]); grid on;
    plot(t, sim_q(idx,:), 'b--', tt, q_cpp(idx,:), 'r-');
    xlabel('time'); ylabel(['q', num2str(idx)]); grid on;
    legend('matlab\_data', 'cpp\_data');
end



end





