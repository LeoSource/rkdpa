clear
close all
clc

addpath('classes');
addpath('tools');
%% define robot model with MDH method
global g_jvmax g_jamax g_cvmax g_camax g_stowed_pos g_cycle_time
g_jvmax = [pi/4, pi/4, pi/4, pi/4, pi/4, pi/4];
g_jamax = [pi/2, pi/2, pi/2, pi/2, pi/2, pi/2];
g_cvmax = [0.15, 0.15]; g_camax = [0.3, 0.3];
g_stowed_pos = [0;0;0;0;-pi/2;0];
g_cycle_time = 0.005;

d1 = 0.048; a2 = 0.51; a3 = 0.51;
d4 = 0.11; d5 = 0.08662; d6 = 0.035;
mdh_table = [0, d1, 0, 0, 0, 0;...
                    0, 0, 0, -pi/2, 0, -pi/2;...
                    0, 0, a2, 0, 0, pi/2;...
                    0, d4, a3, 0, 0, -pi/2;...
                    0, d5, 0, -pi/2, 0, 0;...
                    0, d6, 0, pi/2, 0, 0];
rbt = SerialLink(mdh_table, 'modified', 'name', 'CleanRobot');
pose_tool = SE3;
pose_tool.t = [0,0,0.116];
simu_mode = 'mirror';
switch simu_mode
    case 'mirror'
%% simulation mirror task
q0 = [0,0,0,0,-pi/2,0];
dt = 0.01;
comparision = 0;
pose0 = rbt.fkine(q0)*pose_tool;
pos0 = pose0.t;
rpy0 = tr2rpy(pose0,'xyz');
ctraj = HybridTrajPlanner(pos0,rpy0', 1);
pos1 = [0.5,0,1]'; pos2 = [0.65,0,1]'; pos3 = [0.65,0,0.6]';
rpy1 = [0,pi/2,pi/2]'; rpy2 = [0,pi/2,pi/2]'; rpy3 = [0,pi/2,pi/2]';
ctraj.AddPosRPY([pos1;rpy1]);
ctraj.AddPosRPY([pos2;rpy2]);
ctraj.AddPosRPY([pos3;rpy3]);
[pos,~,~,rpy,~,~] = ctraj.GenerateTraj(dt);

sim_q = []; sim_pos = []; pre_q = q0;
for idx=1:size(pos,2)
    cmd_pose = SE3.rpy(180/pi*rpy(:,idx)','xyz');
    cmd_pose.t = pos(:,idx);
    rot_mount = cmd_pose.R*(pose_tool.R)';
    pos_mount = cmd_pose.t-rot_mount*pose_tool.t;
    pose_mount = rt2tr(rot_mount,pos_mount);
    tmp_q = rbt.ikine(pose_mount, 'q0',pre_q', 'tol',1e-5)';
    sim_q = [sim_q, tmp_q];
    pose_tmp = rbt.fkine(tmp_q)*pose_tool;
    sim_pos = [sim_pos, pose_tmp.t];
    pre_q = tmp_q;
end

%% plot and compare with cpp data
figure
plot2(pos', 'r--'); hold on; plot2(sim_pos', 'k');
plot2([pos1,pos2,pos3]', 'bo'); axis equal; hold off;
grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

t = 0:dt:dt*(size(sim_q,2)-1);
figure
plot(t,sim_q(1,:),'-', t, sim_q(2,:), '--', t, sim_q(3,:), '-.', t, sim_q(4,:), ':', t, sim_q(5,:), '-', t,sim_q(6,:),'k');
grid on; title('joint position'); legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');

if comparision
    q_cpp = load('./data/mirrortask_jpos1.csv');
    q_cpp = reshape(q_cpp, rbt.n, []);
    tt = g_cycle_time*[0:size(q_cpp,2)-1];
    for idx=1:rbt.n
        figure
        plot(t, sim_q(idx,:), 'b--', tt, q_cpp(idx,:), 'r-');
        xlabel('time'); ylabel(['q', num2str(idx)]); grid on;
        legend('matlab\_data', 'cpp\_data');
    end
end





end