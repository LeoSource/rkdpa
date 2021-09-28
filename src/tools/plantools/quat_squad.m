function [val] =  quat_squad(q,s)

L = size(q,2);
val = q(:,1)';

% �����С��0����ȡ����ȷ������̬֮��ȡ���·��
for j=2:L
    C = dot(q(:,j-1),q(:,j));
    if(C<0)
        q(:,j) = -q(:,j);
    end 
end

% ��ʱ����0,1��ֱ�ӷ��������յ�
if s==0 
    val=q(:,1)';
    return;
elseif s==1
    val=q(:,end)';
    return;
end

for j =2:L
    % ȫ��ϸ��ӳ�䵽�ֲ�ϸ��
    alpha=eval_alpha(s,j,L); 
    t= alpha;
    
    if(alpha>0)
        EPS = 1e-9;

        % ����������̬�ĵ���������Χ[-1,1]
        C = dot(q(:,j-1),q(:,j));

        if ((1 - C) <= EPS) 
            % ��̬֮����ڽӽ��������Բ岹
            val=q(:,j-1)'*(1-s)+q(:,j)'*s; 
            val = quatnormalize(val);
	    return;
		end

        if((1 + C) <= EPS)
            % ����̬�нǽӽ�180�������·���������ȷ��
            % ���Ƕ���ת90
            qtemp(1) = q(4,j); qtemp(2) = -q(3,j); qtemp(3)= q(2,j); qtemp(4) = -q(1,j);
            q(:,j) = qtemp';
        end

        % �����м��
        qa = get_intermediate_control_point(j-1,q);
        qap1 = get_intermediate_control_point(j,q);

        % �岹
        qtemp1 = do_slerp(q(:,j-1)', q(:,j)', t);
        qtemp2 = do_slerp(qa, qap1, t);
        squad = do_slerp(qtemp1, qtemp2, 2*t*(1-t));
        val = squad;val = quatnormalize(val);
	return;
    end

end

end

function [qa] = get_intermediate_control_point(j,q)
% �岹�м��
% ����Ϊ�����յ㣬��ֱ�ӷ��ص�ǰֵ
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
% ȫ��ϸ�ֵ��ֲ�ϸ��
k = s*(L-1)+1;

if((i>=k)&&(i<k+1))
    alpha=k-(i-1);
else
    alpha=0;
end

end

function  q_out = do_slerp(q1,q2,t)
% slerp��ֵ
EPS = 1e-9;
C = dot(q1,q2);

if ((1 - C) <= EPS) 
    % ����̬����ӽ�1�����нǷǳ��ӽ��ڣ�������ֱ�߲岹
    q_out=q1*(1-t)+q2*t; % avoiding divisions by number close to 0
    q_out = quatnormalize(q_out);
    return;
elseif((1 + C) <= EPS) 
    % ����̬����ӽ�-1�����нǷǳ��ӽ���18��
    qtemp(1) = q2(4); qtemp(2) = -q2(3); qtemp(3)= q2(2); qtemp(4) = -q2(1); % rotating one of the unit quaternions by 90 degrees -> q2
    q2 =qtemp;
end
% �岹
q1_inv    = quatinv(q1);
q1_inv_q2 = quatmultiply(q1_inv,q2);
omega     = quatlog(q1_inv_q2);
q_out = quatmultiply(q1,quatexp(omega*t));
%q_out = quatnormalize(q_out);
end
