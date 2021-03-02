function q = IKSolve(frame, option, alpha)

    %%to guarantee that the end-effector's pose is perpendicular to
    %%vertical surface, but it is very hard to get a analytical value
    q = zeros(1,5);
    height_limit = [0.5, 1.5];
    [~, pos] = tr2rt(frame);
    ty = 0.2; tz = 0;
    switch option
        case 'vertical'
            if pos(3)>height_limit(2)
                q(2) = height_limit(2);
                q(3) = atan((pos(3)-q(2))/pos(2));%there is problem to calculate q3
                q(1) = -atan(pos(1)/(pos(2)-ty*cos(q(3))));%there is problem to calculate q1
                a = sin(q(1))*cos(q(3)); b = cos(q(1)); c= sin(q(1))*sin(q(3))*tz/ty;
                if abs(a+c)<eps
                    q(5) = 0;
                else
                    q5_tmp1 = 2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));
                    q5_tmp2 = 2*atan((b-sqrt(b^2+a^2-c^2))/(a+c));
                    if (q5_tmp1>pi/2) || (q5_tmp1<-pi/2)
                        q(5) = q5_tmp2;
                    else
                        q(5) = q5_tmp1;
                    end
                end
                tmp_value = pos(3)-q(2)-sin(q(3))*cos(q(5))*ty+cos(q(3))*tz;
                q(4) = tmp_value/sin(q(3));
            elseif pos(3)<height_limit(1)
                q(2) = height_limit(1);
                q(3) = atan((pos(3)-q(2))/pos(2));
                q(1) = -atan(pos(1)/(pos(2)-ty*cos(q(3))));
                a = sin(q(1))*cos(q(3)); b = cos(q(1)); c= sin(q(1))*sin(q(3))*tz/ty;
                if abs(a+c)<eps
                    q(5) = 0;
                else
                    q5_tmp1 = 2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));
                    q5_tmp2 = 2*atan((b-sqrt(b^2+a^2-c^2))/(a+c));
                    if (q5_tmp1>pi/2) || (q5_tmp1<-pi/2)
                        q(5) = q5_tmp2;
                    else
                        q(5) = q5_tmp1;
                    end
                end
                tmp_value = pos(3)-q(2)-sin(q(3))*cos(q(5))*ty+cos(q(3))*tz;
                q(4) = tmp_value/sin(q(3));
            else
                q(2) = pos(3);
                q(3) = 0;
                q(1) = -atan(pos(1)/(pos(2)-ty));
                if abs(q(1))<eps
                    q(4) = pos(2)-ty;
                else
                    q(4) = -pos(1)/sin(q(1));
                end
                q(5) = -q(1);
            end        
            
        case 'horizontal'
            q(3) = -pi/6;
            q(1) = -atan(pos(1)/(pos(2)-cos(q(3))*ty));
            if abs(q(1))<eps
                q(4) = pos(2)/cos(q(3))-ty;
            else
                q(4) = -pos(1)/(sin(q(1))*cos(q(3)));
            end
            a = sin(q(1))*cos(q(3)); b = cos(q(1)); c= sin(q(1))*sin(q(3))*tz/ty;
                if abs(a+c)<eps
                    q(5) = 0;
                else
                    q5_tmp1 = 2*atan((b+sqrt(b^2+a^2-c^2))/(a+c));
                    q5_tmp2 = 2*atan((b-sqrt(b^2+a^2-c^2))/(a+c));
                    if (q5_tmp1>pi/2) || (q5_tmp1<-pi/2)
                        q(5) = q5_tmp2;
                    else
                        q(5) = q5_tmp1;
                    end
                end
            q(2) = pos(3)-sin(q(3))*q(4) - sin(q(3))*cos(q(5))*ty - cos(q(3))*tz;
        
        case 'circle'
            circle_option = 'slope';
            q = IKSolveCircle(pos, circle_option, alpha);
            
        otherwise
            error('IKSolver dose not work');
    end

end

