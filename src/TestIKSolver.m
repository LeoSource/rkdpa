clear
close all
clc

%% tests for iksolver
% to do: random position to test
q = IKSolveSimple([-0.7, 0.7, 1.8], 'vertical', 0);
rot = tr2rt(rbt.fkine(q));
pos = rbt.fkine(q).t + rot*[0;0.2;0]


q2 = IKSolveSimple([-0.8, 0.8, 0.4], 'horizontal', 0);
rot = tr2rt(rbt.fkine(q2));
pos = rbt.fkine(q2).t + rot*[0;0.2;0]


pos = [0, 0.7989, 0.4739];
circle_params.origin = [0; 0.5; 0.5];
circle_params.radius = radius_store(idx);
circle_params.alpha = alpha(idx);