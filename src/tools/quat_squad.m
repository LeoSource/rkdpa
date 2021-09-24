function [val] =  quat_squad(q,s)

L = size(q,2);
val = q(:,1)';

% 若点积小于0进行取负，确保两姿态之间取最短路径
for j=2:L
    C = dot(q(:,j-1),q(:,j));
    if(C<0)
        q(:,j) = -q(:,j);
    end 
end

% 若时间在0,1则直接返回起点和终点
if s==0 
    val=q(:,1)';
    return;
elseif s==1
    val=q(:,end)';
    return;
end

for j =2:L
    % 全局细分映射到局部细分
    alpha=eval_alpha(s,j,L); 
    t= alpha;
    
    if(alpha>0)
        EPS = 1e-9;

        % 计算两个姿态的点积，结果范围[-1,1]
        C = dot(q(:,j-1),q(:,j));

        if ((1 - C) <= EPS) 
            % 姿态之间过于接近采用线性插补
            val=q(:,j-1)'*(1-s)+q(:,j)'*s; 
            val = quatnormalize(val);
	    return;
		end

        if((1 + C) <= EPS)
            % 当姿态夹角接近180，无最短路径，结果不确定
            % 将角度旋转90
            qtemp(1) = q(4,j); qtemp(2) = -q(3,j); qtemp(3)= q(2,j); qtemp(4) = -q(1,j);
            q(:,j) = qtemp';
        end

        % 计算中间点
        qa = get_intermediate_control_point(j-1,q);
        qap1 = get_intermediate_control_point(j,q);

        % 插补
        qtemp1 = do_slerp(q(:,j-1)', q(:,j)', t);
        qtemp2 = do_slerp(qa, qap1, t);
        squad = do_slerp(qtemp1, qtemp2, 2*t*(1-t));
        val = squad;val = quatnormalize(val);
	return;
    end

end

end

function [qa] = get_intermediate_control_point(j,q)
% 插补中间点
% 若点为起点和终点，则直接返回当前值
L = size(q,2);
if(j==1)
    qa =  q(:,1)';
    return;
elseif(j==L)
    qa =  q(:,L)';
    return;
else
    qji=quatinv(q(:,j)');
    qiqm1=quatmultiply(qji,q(:,j-1)');
    qiqp1=quatmultiply(qji,q(:,j+1)');
    ang_vel =-((quatlog(qiqp1)+quatlog(qiqm1))/4); 
    
    qa = quatmultiply(q(:,j)',quatexp(ang_vel));
    %qa = quatnormalize(qa);
end
end

function alpha = eval_alpha(s,i,L)
% 全局细分到局部细分
k = s*(L-1)+1;

if((i>=k)&&(i<k+1))
    alpha=k-(i-1);
else
    alpha=0;
end

end

function  q_out = do_slerp(q1,q2,t)
% slerp插值
EPS = 1e-9;
C = dot(q1,q2);

if ((1 - C) <= EPS) 
    % 两姿态点积接近1，即夹角非常接近于０，采用直线插补
    q_out=q1*(1-t)+q2*t; % avoiding divisions by number close to 0
    q_out = quatnormalize(q_out);
    return;
elseif((1 + C) <= EPS) 
    % 两姿态点积接近-1，即夹角非常接近于18０
    qtemp(1) = q2(4); qtemp(2) = -q2(3); qtemp(3)= q2(2); qtemp(4) = -q2(1); % rotating one of the unit quaternions by 90 degrees -> q2
    q2 =qtemp;
end
% 插补
q1_inv    = quatinv(q1);
q1_inv_q2 = quatmultiply(q1_inv,q2);
omega     = quatlog(q1_inv_q2);
q_out = quatmultiply(q1,quatexp(omega*t));
%q_out = quatnormalize(q_out);
end
