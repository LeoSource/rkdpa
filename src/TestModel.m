clear
close all
clc

rbt = CleanRobot;
test_mode = 'workspace';
if strcmp(test_mode, 'dhmodel')
%% validation for robot model by simscape
%%there is lag in Simulink_PS Converter block because of the filter
sample_time = 0.001;
tf = 10;
t = [0:sample_time:tf]';
q(:,1) = pi/2*sin(2*pi*0.5*t);
q(:,2) = 0.5*sin(2*pi*0.5*t);
q(:,3) = pi/2*sin(2*pi*0.5*t);
q(:,4) =  0.5*sin(2*pi*0.5*t);
q(:,5) = pi/2*sin(2*pi*0.5*t);
q_offset = [0, 0.75, 0, 0, 0];
cart_pos = [];
for idx=1:size(t,1)
    q_in = q(idx,:) + q_offset;
    tmp_frame = rbt.FKSolve(q_in);
    end_rot = [tmp_frame.n, tmp_frame.o, tmp_frame.a];
    cart_pos = [cart_pos, tmp_frame.t];
end
sim('test_model');
sim_pos(:,1) = sim_pos_x;
sim_pos(:,2) = sim_pos_y;
sim_pos(:,3) = sim_pos_z;
%figure
figure(1)
subplot(2,1,1)
plot(t,sim_pos_x ,t, cart_pos(1,:),'--');
xlabel('time(s)');
ylabel('pos_x(m)');
legend('simscape model', 'mdh model');
grid on
subplot(2,1,2)
plot(t, sim_pos_x - cart_pos(1,:)');
legend('position error');

figure(2)
subplot(2,1,1)
plot(t,sim_pos_y ,t, cart_pos(2,:),'--');
xlabel('time(s)');
ylabel('pos_y(m)');
legend('simscape model', 'mdh model');
grid on
subplot(2,1,2)
plot(t, sim_pos_y - cart_pos(2,:)');
legend('position error');

figure(3)
subplot(2,1,1)
plot(t,sim_pos_z ,t, cart_pos(3,:),'--');
xlabel('time(s)');
ylabel('pos_z(m)');
legend('simscape model', 'mdh model');
grid on
subplot(2,1,2)
plot(t, sim_pos_z - cart_pos(3,:)');
legend('position error');


elseif strcmp(test_mode, 'workspace')
%% validation for the workspace of cleanrobot    
rbt.PlotWorkspace;

end
