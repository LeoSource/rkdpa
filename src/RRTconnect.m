% ==============================================================================
%
%    Software License Agreement (BSD License)
%    Copyright (c) 2019
%    (www.aimlab.wpi.edu)
%
%    All rights reserved.
%
%    Redistribution and use in source and binary forms, with or without
%    modification, are permitted provided that the following conditions
%    are met:
%
%    * Redistributions of source code must retain the above copyright
%    notice, this list of conditions and the following disclaimer.
%
%    * Redistributions in binary form must reproduce the above
%    copyright notice, this list of conditions and the following
%    disclaimer in the documentation and/or other materials provided
%    with the distribution.
%
%    * Neither the name of authors nor the names of its contributors may
%    be used to endorse or promote products derived from this software
%    without specific prior written permission.
%
%    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
%    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
%    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
%    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
%    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
%    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
%    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
%    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
%    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%    POSSIBILITY OF SUCH DAMAGE.
%
%    \author:    <http://www.aimlab.wpi.edu>
%    \author:    <amunawar@wpi.edu>
%    \author:    Adnan Munawar
%    \version:   0.1$
% ==============================================================================

function [nIterations,sizePath,run_time] =  RRTconnect(dim,segmentLength,random_world,show_output)
% dim = 2;
% segmentLength = 5;
% random_world = 0;
% standard length of path segments
frame = creatKnownFrame([0,0], [100,100]);
rbt = creatRobot();

q_start = [10, 30, 0]*pi/180;
q_goal = [80, -50, 20]*pi/180;

start_node = [q_start, 0, 0, 0];
end_node = [q_goal, 0, 0, 0];
tree = start_node;
a= clock;

% check to see if start_node connects directly to end_node
if ( (norm(start_node(1:dim)-end_node(1:dim))<segmentLength )...
        &&(robotCollision(frame, rbt, end_node(1:dim))) ==0) 
    path = [start_node; end_node];
else
    nIterations = 0;
    numPaths = 0;
    flag = 0;
    while numPaths<1
        [tree,flag] = extendRandTree(frame, tree, rbt, q_goal, segmentLength);
        numPaths = numPaths + flag;
        nIterations = nIterations+1;
    end   
end

path = findPath(tree, end_node);
sizePath = size(path,1);
b = clock;
run_time = 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6));

plotJntAVP(path, 1);
plotJntAVP(path, 2);
plotJntAVP(path, 3);
% plotRobotMotion(frame, rbt, path, 'original');
plotRobotMotion(frame, rbt, path, 'spline');
plotRobotMotion(frame, rbt, path, 'traj');

return;%return for test

%% original code
if dim ==2
    start_cord = [5,5];
    goal_cord = [95,95];
    
else
    
    start_cord = [5,5,5];
    goal_cord = [95,95,95];
end


% create random world
Size = 100;
NumObstacles = 100;

if random_world ==1
    world = createWorld(NumObstacles,ones(1,dim)*Size,zeros(1,dim),dim);
else
    [world, ~] = createKnownWorld(ones(1,dim)*Size,[0;0;0],dim);
end
% randomly select start and end nodes
%start_node = generateRandomNode(world,dim)
%end_node   = generateRandomNode(world,dim)
start_node = [start_cord,0,0,0];
end_node = [goal_cord,0,0,0];
% establish tree starting with the start node
tree = start_node;

a = clock;

% check to see if start_node connects directly to end_node
if ( (norm(start_node(1:dim)-end_node(1:dim))<segmentLength )...
        &&(collision(start_node,end_node,world,dim)==0) )
    path = [start_node; end_node];
else
    nIterations = 0;
    numPaths = 0;
    flag = 0;
    while numPaths<1
        [tree,flag] = extendTree(tree,end_node,segmentLength,world,dim);
        numPaths = numPaths + flag;
        nIterations = nIterations+1;
%         figure;
%         plotExpandedTree(world,tree,dim);
    end   
end


path = findMinimumPath(tree,end_node,dim);
sizePath = size(path,1);

b = clock;

run_time = 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6));

if show_output == 1
    % find path with minimum cost to end_node
    figure;
    plotExpandedTree(world,tree,dim);
    plotWorld(world,path,dim);
%     figure(2);
%     plotWorld(world,path,dim);
%     plotExpandedTree(world,tree,dim);
end
end





function world = createWorld(NumObstacles, endcorner, origincorner,dim)

if dim == 2
    
    % check to make sure that the region is nonempty
    if (endcorner(1) <= origincorner(1)) | (endcorner(2) <= origincorner(2))
        disp('Not valid corner specifications!')
        world=[];
        
        % create world data structure
    else
        world.NumObstacles = NumObstacles;
        world.endcorner = endcorner;
        world.origincorner = origincorner;
        
        % create NumObstacles
        maxRadius = min(endcorner(1)- origincorner(1), endcorner(2)-origincorner(2));
        maxRadius = 5*maxRadius/NumObstacles/2;
        for i=1:NumObstacles,
            % randomly pick radius
            world.radius(i) = maxRadius*rand;
            % randomly pick center of obstacles
            cx = origincorner(1) + world.radius(i)...
                + (endcorner(1)-origincorner(1)-2*world.radius(i))*rand;
            cy = origincorner(2) + world.radius(i)...
                + (endcorner(2)-origincorner(2)-2*world.radius(i))*rand;
            world.cx(i) = cx;
            world.cy(i) = cy;
        end
    end
    
elseif dim ==3;
    % check to make sure that the region is nonempty
    if (endcorner(1) <= origincorner(1)) || (endcorner(2) <= origincorner(2)) || (endcorner(3) <= origincorner(3))
        disp('Not valid corner specifications!')
        world=[];
        
        % create world data structure
    else
        world.NumObstacles = NumObstacles;
        world.endcorner = endcorner;
        world.origincorner = origincorner;
        
        % create NumObstacles
        bounds = [endcorner(1)- origincorner(1), endcorner(2)-origincorner(2), endcorner(3)-origincorner(3)];
        maxRadius = min(bounds);
        maxRadius = 5*maxRadius/NumObstacles;
        for i=1:NumObstacles,
            % randomly pick radius
            world.radius(i) = maxRadius*rand;
            % randomly pick center of obstacles
            cx = origincorner(1) + world.radius(i)...
                + (endcorner(1)-origincorner(1)-2*world.radius(i))*rand;
            cy = origincorner(2) + world.radius(i)...
                + (endcorner(2)-origincorner(2)-2*world.radius(i))*rand;
            cz = origincorner(2) + world.radius(i)...
                + (endcorner(2)-origincorner(2)-2*world.radius(i))*rand;
            world.cx(i) = cx;
            world.cy(i) = cy;
            world.cz(i) = cz;
        end
    end
end
end

function [world NumObstacles] = createKnownWorld(endcorner, origincorner,dim)
NumObstacles = 5;
if dim == 2
    % check to make sure that the region is nonempty
    if (endcorner(1) <= origincorner(1)) | (endcorner(2) <= origincorner(2)),
        disp('Not valid corner specifications!')
        world=[];
        % create world data structure
    else
        world.NumObstacles = NumObstacles;
        world.endcorner = endcorner;
        world.origincorner = origincorner;
        
        % create NumObstacles
        maxRadius = 10;
        
        world.radius(1) = maxRadius;
        cx = 50;
        cy = 50;
        world.cx(1) = cx;
        world.cy(1) = cy;
        
        world.radius(2) = maxRadius;
        cx = 75;
        cy = 25;
        world.cx(2) = cx;
        world.cy(2) = cy;
        
        world.radius(3) = maxRadius;
        cx = 25;
        cy = 75;
        world.cx(3) = cx;
        world.cy(3) = cy;
        
        world.radius(4) = maxRadius;
        cx = 25;
        cy = 25;
        world.cx(4) = cx;
        world.cy(4) = cy;
        
        world.radius(5) = maxRadius;
        cx = 75;
        cy = 75;
        world.cx(5) = cx;
        world.cy(5) = cy;
    end
    
elseif dim == 3
    
    NumObstacles = 9;
    % check to make sure that the region is nonempty
    if (endcorner(1) <= origincorner(1)) | (endcorner(2) <= origincorner(2)) | (endcorner(3) <= origincorner(3)),
        disp('Not valid corner specifications!')
        world=[];
        
        % create world data structure
    else
        world.NumObstacles = NumObstacles;
        world.endcorner = endcorner;
        world.origincorner = origincorner;
        
        % create NumObstacles
        maxRadius = 10;
        
        world.radius(1) = maxRadius;
        cx = 50;
        cy = 50;
        cz = 50;
        world.cx(1) = cx;
        world.cy(1) = cy;
        world.cz(1) = cz;
        
        world.radius(2) = maxRadius;
        cx = 25;
        cy = 25;
        cz = 25;
        world.cx(2) = cx;
        world.cy(2) = cy;
        world.cz(2) = cz;
        
        world.radius(3) = maxRadius;
        cx = 75;
        cy = 75;
        cz = 75;
        world.cx(3) = cx;
        world.cy(3) = cy;
        world.cz(3) = cz;
        
        world.radius(4) = maxRadius;
        cx = 25;
        cy = 25;
        cz = 75;
        world.cx(4) = cx;
        world.cy(4) = cy;
        world.cz(4) = cz;
        
        world.radius(5) = maxRadius;
        cx = 75;
        cy = 75;
        cz = 25;
        world.cx(5) = cx;
        world.cy(5) = cy;
        world.cz(5) = cz;
        
        world.radius(6) = maxRadius;
        cx = 25;
        cy = 75;
        cz = 25;
        world.cx(6) = cx;
        world.cy(6) = cy;
        world.cz(6) = cz;
        
        world.radius(7) = maxRadius;
        cx = 75;
        cy = 25;
        cz = 25;
        world.cx(7) = cx;
        world.cy(7) = cy;
        world.cz(7) = cz;
        
        world.radius(8) = maxRadius;
        cx = 75;
        cy = 25;
        cz = 75;
        world.cx(8) = cx;
        world.cy(8) = cy;
        world.cz(8) = cz;
        
        
        world.radius(9) = maxRadius;
        cx = 25;
        cy = 75;
        cz = 75;
        world.cx(9) = cx;
        world.cy(9) = cy;
        world.cz(9) = cz;
    end
end
end





function node=generateRandomNode(world,dim)

if dim ==2;
    % randomly pick configuration
    px       = (world.endcorner(1)-world.origincorner(1))*rand;
    py       = (world.endcorner(2)-world.origincorner(2))*rand;
    
    chi      = 0;
    cost     = 0;
    node     = [px, py, chi, cost, 0];
    
    % check collision with obstacle
    while collision(node, node, world,dim),
        px       = (world.endcorner(1)-world.origincorner(1))*rand;
        py       = (world.endcorner(2)-world.origincorner(2))*rand;
        
        chi      = 0;
        cost     = 0;
        node     = [px, py, chi, cost, 0];
    end
    
elseif dim ==3;
    % randomly pick configuration
    px       = (world.endcorner(1)-world.origincorner(1))*rand;
    py       = (world.endcorner(2)-world.origincorner(2))*rand;
    pz       = (world.endcorner(3)-world.origincorner(3))*rand;
    
    chi      = 0;
    cost     = 0;
    node     = [px, py, pz, chi, cost, 0];
    
    % check collision with obstacle
    while collision(node, node, world,dim),
        px       = (world.endcorner(1)-world.origincorner(1))*rand;
        py       = (world.endcorner(2)-world.origincorner(2))*rand;
        pz       = (world.endcorner(3)-world.origincorner(3))*rand;
        
        chi      = 0;
        cost     = 0;
        node     = [px, py, pz, chi, cost, 0];
    end
    
end

end





function collision_flag = collision(node, parent, world,dim)

collision_flag = 0;


for i=1:dim
    if (node(i)>world.endcorner(i))|(node(i)<world.origincorner(i))
        collision_flag = 1;
    end
end

if collision_flag == 0 && dim ==2
    for sigma = 0:.2:1,
        p = sigma*node(1:dim) + (1-sigma)*parent(1:dim);
        % check each obstacle
        for i=1:world.NumObstacles,
            if (norm([p(1);p(2)]-[world.cx(i); world.cy(i)])<=1*world.radius(i)),
                collision_flag = 1;
                break;
            end
        end
    end
    
elseif collision_flag == 0 && dim ==3
    for sigma = 0:.2:1,
        p = sigma*node(1:dim) + (1-sigma)*parent(1:dim);
        % check each obstacle
        for i=1:world.NumObstacles,
            if (norm([p(1);p(2);p(3)]-[world.cx(i); world.cy(i); world.cz(i)])<=1*world.radius(i)),
                collision_flag = 1;
                break;
            end
        end
    end
end
end



function collision_flag = is_point_valid(point, world,dim)

collision_flag = 0;


for i=1:dim
    if (point(i)>world.endcorner(i))||(point(i)<world.origincorner(i))
        collision_flag = 1;
    end
end

if collision_flag == 0 && dim ==2
    
    p = point(1:dim);
    % check each obstacle
    for i=1:world.NumObstacles,
        if (norm([p(1);p(2)]-[world.cx(i); world.cy(i)])<=1*world.radius(i)),
            collision_flag = 1;
            break;
        end
    end
    
elseif collision_flag == 0 && dim ==3
    p = point(1:dim);
    % check each obstacle
    for i=1:world.NumObstacles,
        if (norm([p(1);p(2);p(3)]-[world.cx(i); world.cy(i); world.cz(i)])<=1*world.radius(i)),
            collision_flag = 1;
            break;
        end
    end
end
end







function flag = canEndConnectToTree(tree,end_node,minDist,world,dim)
flag = 0;
% check only last node added to tree since others have been checked
if ( (norm(tree(end,1:dim)-end_node(1:dim))<minDist)...
        & (collision(tree(end,1:dim), end_node(1:dim), world,dim)==0) ),
    flag = 1;
end

end








function [new_tree,flag] = extendTree(tree,end_node,segmentLength,world,dim)

flag = 0;
% select a random point
randomPoint = zeros(1,dim);
for i=1:dim
    randomPoint(1,i) = (world.endcorner(i)-world.origincorner(i))*rand;
end



% find leaf on node that is closest to randomPoint
tmp = tree(:,1:dim)-ones(size(tree,1),1)*randomPoint;
sqrd_dist = sqr_eucl_dist(tmp,dim);
[min_dist,idx] = min(sqrd_dist);

min_parent_idx = idx;

new_point = tree(idx,1:dim);
new_node = tree(idx,:);

pflag = 0;


while norm(new_point-randomPoint)>0 && pflag==0
    
    
    if norm(new_point-randomPoint)<segmentLength
        
        pflag = collision(randomPoint,tree(min_parent_idx,:),world,dim);
        if pflag == 0
            
            new_point = randomPoint;
            min_cost = cost_np(tree(min_parent_idx,:),new_point,dim);
            new_node = [new_point,0,min_cost,min_parent_idx];
            tree = [tree;new_node];
            pflag = 1;
            goal_flag = is_goal(new_node,end_node,segmentLength,world,dim);
            
            if goal_flag == 1
                tree(end,dim+1)=1;
                flag = 1;
            end
        end
        
    else
        
        new_point = (randomPoint-tree(min_parent_idx,1:dim));
        new_point = tree(min_parent_idx,1:dim)+(new_point/norm(new_point))*segmentLength;
        
        min_cost  = cost_np(tree(min_parent_idx,:),new_point,dim);
        new_node  = [new_point, 0, min_cost, min_parent_idx];
        
        pflag = collision(new_node,tree(min_parent_idx,:),world,dim);
        
        if pflag == 0
            tree = [tree ; new_node];
            min_parent_idx = size(tree,1);
            
            goal_flag = is_goal(new_node,end_node,segmentLength,world,dim);
            
            if goal_flag == 1
                tree(end,dim+1)=1;  % mark node as connecting to end.
                pflag = 1;
                flag = 1;
            end
        end
    end
end

new_tree = tree;

end




function goal_flag = is_goal(node,end_node,segmentLength,world,dim)

goal_flag = 0;
if (norm(node(1:dim)-end_node(1:dim))<segmentLength )...
        && (collision(node,end_node,world,dim)==0)
    goal_flag = 1;
end

end


function e_dist = sqr_eucl_dist(array,dim)

sqr_e_dist = zeros(size(array,1),dim);
for i=1:dim
    
    sqr_e_dist(:,i) = array(:,i).*array(:,i);
    
end
e_dist = zeros(size(array,1),1);
for i=1:dim
    
    e_dist = e_dist+sqr_e_dist(:,i);
    
end

end



%calculate the cost from a node to a point
function [cost] = cost_np(from_node,to_point,dim)

diff = from_node(:,1:dim) - to_point;
eucl_dist = norm(diff);
cost = from_node(:,dim+2) + eucl_dist;

end


%calculate the cost from a node to a node
function [cost] = cost_nn(from_node,to_node,dim)

diff = from_node(:,1:dim) - to_node(:,1:dim);
eucl_dist = norm(diff);
cost = from_node(:,dim+2) + eucl_dist;

end

function [cost] = line_cost(from_node,to_point,dim)
diff = from_node(:,1:dim) - to_point;
cost = norm(diff);
end


function path = findMinimumPath(tree,end_node,dim)

% find nodes that connect to end_node
connectingNodes = [];
for i=1:size(tree,1)
    if tree(i,dim+1)==1
        connectingNodes = [connectingNodes ; tree(i,:)];
    end
end

% find minimum cost last node
[tmp,idx] = min(connectingNodes(:,dim+2));

% construct lowest cost path
path = [connectingNodes(idx,:); end_node];
parent_node = connectingNodes(idx,dim+3);
while parent_node>1
    parent_node = tree(parent_node,dim+3);
    path = [tree(parent_node,:); path];
end

end


function plotExpandedTree(world,tree,dim)
ind = size(tree,1);
while ind>0
    size(tree);
    branch = [];
    node = tree(ind,:);
    branch = [ branch ; node ];
    parent_node = node(dim+3);
    while parent_node > 1
        cur_parent = parent_node;
        branch = [branch; tree(parent_node,:)];
        parent_node = tree(parent_node,dim+3);
    end
    ind = ind - 1;
    
    if dim == 2
        X = branch(:,1);
        Y = branch(:,2);
        
        p = plot(X,Y);
        set(p,'Color','r','LineWidth',0.5,'Marker','.','MarkerEdgeColor','g');
        hold on;
        
    elseif dim == 3
        X = branch(:,1);
        Y = branch(:,2);
        Z = branch(:,3);
        
        p = plot3(X,Y,Z);
        set(p,'Color','r','LineWidth',0.5,'Marker','.','MarkerEdgeColor','g');
        hold on;
    end
end
end




function plotWorld(world,path,dim)
% the first element is the north coordinate
% the second element is the south coordinate
if dim ==2
    
    N = 20;
    th = 0:2*pi/N:2*pi;
    axis([world.origincorner(1),world.endcorner(1),...
        world.origincorner(2), world.endcorner(2)]);
    hold on
    
    for i=1:world.NumObstacles,
        X = world.radius(i)*sin(th) + world.cx(i);
        Y = world.radius(i)*cos(th) + world.cy(i);
        fill(X,Y,'black');
    end
    
    X = path(:,1);
    Y = path(:,2);
    p = plot(X,Y);
    
elseif dim ==3
    axis([world.origincorner(1),world.endcorner(1),...
        world.origincorner(2), world.endcorner(2),...
        world.origincorner(3), world.endcorner(3)]);
    hold on
    
    for i=1:world.NumObstacles,
        [X Y Z] = sphere(10);
        X = (X*world.radius(i));
        Y = (Y*world.radius(i));
        Z = (Z*world.radius(i));
        surf(X+world.cx(i),Y+world.cy(i),Z+world.cz(i));
        colormap([0.5 0.2 0.3]);
    end
    
    X = path(:,1);
    Y = path(:,2);
    Z = path(:,3);
    p = plot3(X,Y,Z);
end
set(p,'Color','black','LineWidth',3)
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
title('RRT Connect Algorithm');
end


%% to test two-links robot
function frame = creatKnownFrame(origincorner, endcorner)
    frame.origincorner = origincorner;
    frame.endcorner = endcorner;
    frame.numobstacle = 2;
    
    frame.obstacle(1).origin = [60; 50];
    frame.obstacle(1).radius = 10;
    
    frame.obstacle(2).origin = [10; 70];
    frame.obstacle(2).radius = 10;
    
end

function rbt = creatRobot()
    rbt.base = [0; 0];
    rbt.numlinks = 3;
    rbt.linklength = [30, 30, 30];
    rbt.qlim = [0, pi/2; -pi, pi; -pi, pi];
end

function plotFrame(frame)
    num = 20;
    alpha = linspace(0,2*pi,num);
    for idx=1:frame.numobstacle
        origin = frame.obstacle(idx).origin;
        radius = frame.obstacle(idx).radius;
        x = origin(1)+cos(alpha)*radius;
        y = origin(2)+sin(alpha)*radius;
        fill(x, y, 'black');
        hold on
    end
    axis([frame.origincorner(1),frame.endcorner(1),...
        frame.origincorner(2), frame.endcorner(2)]);
end

function plotRobot(rbt, q)
    base = rbt.base;
    pos = fk(rbt, q);
    plot([base(1), pos{1}(1), pos{2}(1), pos{3}(1)], [base(2), pos{1}(2), pos{2}(2), pos{3}(2)], 'k', 'LineWidth', 3);
end

function plotJntAVP(path, idx)
    figure;
    dt = 0.1;
    subplot(3,1,1)
    t = 0:dt:size(path,1)-1;
    q = interp1([0:size(path,1)-1], path(:,idx)*180/pi, t, 'spline');
    planner = TrajPlanner(path(:,idx)*180/pi, size(path,1)-1);
    [q_traj, vel_traj, acc_traj] = planner.GenerateTraj(dt);
    plot([0:size(path,1)-1], path(:,idx)*180/pi, 'o', t, q, 'b--', t, q_traj, 'r-');
    ylabel('pos(degree)');
    grid on
    subplot(3,1,2)
    dq = diff(q)/dt;
    plot(t(1:end-1), dq, 'b--', t, vel_traj, 'r-');
    ylabel('vel(degree/s)');
    grid on
    subplot(3,1,3)
    ddq = diff(dq)/dt;
    plot(t(1:end-2), ddq, 'b--', t, acc_traj, 'r-');
    ylabel('acc(degree/s^2)');
    grid on
end

function plotRobotMotion(frame, rbt, path, option)
    dt = 0.1;
    t = 0:dt:size(path,1)-1;
    if strcmp(option, 'original')
        num = size(path,1);
        q = path(:,1:rbt.numlinks);
    elseif strcmp(option, 'spline')
        num = length(t);
        q = interp1([0:size(path,1)-1], path(:,1:rbt.numlinks), t, 'spline');
    else
        num = length(t);
        for idx=1:rbt.numlinks
            traj_planner(idx) = TrajPlanner(path(:,idx), size(path,1)-1);
            q(idx,:) = traj_planner(idx).GenerateTraj(dt);
        end
        q = q';
    end
    M = moviein(60);
    figure;
    pic_num = 1;
    for idx=1:num
        plotFrame(frame);
        hold on
        plotRobot(rbt, q(idx,:));
        hold off
        M(:,end+1) = getframe;
        
        I = frame2im(M(:,end));
        [I, map] = rgb2ind(I, 256);
        if pic_num==1
            imwrite(I,map, 'RobotMotion.gif', 'gif', 'Loopcount', inf, 'DelayTime', 0.02);
        else
            imwrite(I, map, 'RobotMotion.gif', 'gif', 'WriteMode', 'append', 'DelayTime', 0.02);
        end
        pic_num = pic_num+1;
    end
%     movie(M,1,5);
end

function pos = fk(rbt, q)
    base = rbt.base;
    linklength = rbt.linklength;
    
    pos{1} = [cos(q(1))*linklength(1); sin(q(1))*linklength(1)] + base;
    pos{2} = pos{1}+[cos(q(1)+q(2))*linklength(2); sin(q(1)+q(2))*linklength(2)];
    pos{3} = pos{2}+[cos(sum(q))*linklength(3); sin(sum(q))*linklength(3)];
end

function collision = robotCollision(frame, rbt, q)
    collision = 0;
    numlinks = rbt.numlinks;
    pos = fk(rbt, q);
    for lidx=1:numlinks
        if q(lidx)<rbt.qlim(lidx,1) || q(lidx)>rbt.qlim(lidx,2) ||...
                pos{lidx}(1)<frame.origincorner(1) || pos{lidx}(1)>frame.endcorner(1) ||...
                pos{lidx}(2)<frame.origincorner(2) || pos{lidx}(2)>frame.endcorner(2)
            collision = collision+1;
        end
    end
    
    dis_tol = 2;
    if collision==0
        for dt=0:0.2:1
            p = rbt.base+(pos{1}-rbt.base)*dt;
            for idx=1:frame.numobstacle
                dis = norm(p-frame.obstacle(idx).origin);
                if dis<(frame.obstacle(idx).radius+dis_tol)
                    collision = collision+1;
                    break;
                end
            end
            
            p = pos{1}+(pos{2}-pos{1})*dt;
            for idx=1:frame.numobstacle
                dis = norm(p-frame.obstacle(idx).origin);
                if dis<(frame.obstacle(idx).radius+dis_tol)
                    collision = collision+1;
                    break;
                end
            end            
            
            p = pos{2}+(pos{3}-pos{2})*dt;
            for idx=1:frame.numobstacle
                dis = norm(p-frame.obstacle(idx).origin);
                if dis<(frame.obstacle(idx).radius+dis_tol)
                    collision = collision+1;
                    break;
                end
            end
        end
        
    end
        
end

function [new_tree, flag] = extendRandTree(frame, tree, rbt, q_goal, segmentLength)
    flag = 0;
    dim = 3;
    % select a random point
    randq = rbt.qlim(:,1)+(rbt.qlim(:,2)-rbt.qlim(:,1)).*rand(rbt.numlinks,1);
    randq = randq';

    % find leaf on node that is closest to randomPoint
    tmp = tree(:,1:dim)-ones(size(tree,1),1)*randq;
    sqrd_dist = sqr_eucl_dist(tmp,dim);
    [~, idx] = min(sqrd_dist);
    
    min_parent_idx = idx;
    new_q = tree(idx,1:dim);
    new_node = tree(idx,:);
    
    pflag = 0;
    while norm(new_q-randq)>0 && pflag==0
        
        if norm(new_q-randq)<segmentLength
            pflag = robotCollision(frame, rbt, randq);
            if pflag ==0
                new_q = randq;
                min_cost  = cost_np(tree(min_parent_idx,:),new_q,dim);
                new_node = [new_q,0,min_cost,min_parent_idx];
                tree = [tree; new_node];
                pflag = 1;
                goal_flag = arriveGoal(new_q, q_goal, segmentLength);
                if goal_flag==1
                    tree(end,dim+1) = 1;
                    flag = 1;
                end
            end
        else
            vec = randq-tree(min_parent_idx,1:dim);
            new_q = tree(min_parent_idx,1:dim)+vec/norm(vec)*segmentLength;           
            min_cost  = cost_np(tree(min_parent_idx,:),new_q,dim);
            new_node  = [new_q, 0, min_cost, min_parent_idx];            
            
            pflag = robotCollision(frame, rbt, new_q);
            if pflag==0
                tree = [tree; new_node];
                min_parent_idx = size(tree,1);
                goal_flag = arriveGoal(new_q, q_goal, segmentLength);
                if goal_flag==1
                    tree(end,dim+1) = 1;
                    flag = 1;
                end
            end
        end     
    end
    
    new_tree = tree;
end

function goal_flag = arriveGoal(q, q_goal, segmentLength)
    goal_flag = 0;
    if norm(q-q_goal)<segmentLength
        goal_flag = 1;
    end
end

function path = findPath(tree, end_node)

    dim = 3;
    connectingnodes = [];
    for i=1:size(tree,1)
        if tree(i,dim+1)==1
            connectingnodes = [connectingnodes; tree(i,:)];
        end
    end
    [~, idx] = min(connectingnodes(:,dim+2));
    
    path = [connectingnodes(idx,:); end_node];
    parent_node_idx = connectingnodes(idx, dim+3);
    while parent_node_idx>1
        parent_node_idx = tree(parent_node_idx, dim+3);
        path = [tree(parent_node_idx,:); path];
    end

end