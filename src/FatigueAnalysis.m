clear
close all
clc


dt = 0.005;
rbt = CreateRobot();
td_rbt = load('./data/fatigue_test0222_174543.csv');
td_vision = textread('./data/res.txt');

if mod(size(td_rbt,1),2)==1
    jpos1 = td_rbt(1:2:end,1:6);
    jpos2 = td_rbt(2:2:end-1,1:6);
else
    jpos1 = td_rbt(1:2:end-1,1:6);
    jpos2 = td_rbt(2:2:end,1:6);
end
if mod(size(td_vision,1),2)==1
    td_vision1 = td_vision(1:2:end,:);
    td_vision2 = td_vision(2:2:end-1,:);
else
    td_vision1 = td_vision(1:2:end-1,:);
    td_vision2 = td_vision(2:2:end,:);
end
pose1 = rbt.fkine(jpos1(2,:))
pose2 = rbt.fkine(jpos2(1,:))
for idx=1:size(td_vision1,1)
    pos_tmp = td_vision1(idx,:);
    cpos_tmp = reshape(pos_tmp,3,[]);
    for nidx=1:size(cpos_tmp,2)
        cpos1(:,nidx) = pose1.t+pose1.R*cpos_tmp(:,nidx);
    end
    cpos1_plot{idx} = cpos1;
    pos1_target(idx,:) = reshape(cpos1,1,[]);
end
for idx=1:size(td_vision2,1)
    cpos_tmp = td_vision2(idx,:);
    cpos_tmp = reshape(cpos_tmp,3,[]);
    for nidx=1:size(cpos_tmp,2)
        cpos2(:,nidx) = pose2.t+pose2.R*cpos_tmp(:,nidx);
    end
    cpos2_plot{idx} = cpos2;
    pos2_target(idx,:) = reshape(cpos2,1,[]);
end

figure
for idx=1:200
    plot2(cpos1_plot{idx}','r*'); hold on;
end
for idx=1:200
    plot2(cpos2_plot{idx}','ko'); hold on;
end
axis equal; grid on; xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');

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
    tool_toiletlid = SE3(rotx(0), [0,0,0]);
    qmin = [-pi, -pi/2, -4*pi/3, -pi, -pi, -2*pi]';
    qmax = [pi, pi/2, pi/3, pi, pi, 2*pi]';
    rbt = SerialLink(mdh_table, 'modified', 'name', 'CleanRobot', 'tool',tool_toiletlid);
    rbt.qlim(:,1) = qmin; rbt.qlim(:,2) = qmax;
end