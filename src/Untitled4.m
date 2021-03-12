% clear
% close all
% clc


planner = TrajPlanner(path(:,1), 10);
% params = planner.poly_params;
% pos = []; vel = [];
% for t=0:0.01:4
% %     if t<1
% %         p = params(1,1)+params(2,1)*t+params(3,1)*t^2+params(4,1)*t^3;
% %         v = params(2,1)+2*params(3,1)*t+3*params(4,1)*t^2;
% %     elseif t>=1 && t< 2
% %         p = params(1,2)+params(2,2)*t+params(3,2)*t^2+params(4,2)*t^3;
% %         v = params(2,2)+2*params(3,2)*t+3*params(4,2)*t^2;
% %     elseif t>=2 && t<3
% %         p = params(1,3)+params(2,3)*t+params(3,3)*t^2+params(4,3)*t^3;
% %         v = params(2,3)+2*params(3,3)*t+3*params(4,3)*t^2;
% %     else
% %         p = params(1,4)+params(2,4)*t+params(3,4)*t^2+params(4,4)*t^3;
% %         v = params(2,4)+2*params(3,4)*t+3*params(4,4)*t^2;
% %     end
%     [p, v] = planner.GenerateMotionState(t);
%     pos = [pos ,p];
%     vel = [vel , v];
% end
[pos, vel] = planner.GenerateTraj();
plot(pos);
grid on

figure
plot(vel)
grid on