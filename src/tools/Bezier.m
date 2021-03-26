function Bezier( vertices )
% BEZIER ���ɱ���������

    Dim=size(vertices,1);
    NumPoint=size(vertices,2)-1;
    t=0:0.001:1;
    x=[];y=[];z=[];

    if Dim==2
         x=(1-t).^(NumPoint)*vertices(1,1);
         y=(1-t).^(NumPoint)*vertices(2,1);
      for j=1:NumPoint
          w=factorial(NumPoint)/(factorial(j)*factorial(NumPoint-j))*(1-t).^(NumPoint-j).*t.^(j);
          x=x+w*vertices(1,j+1);y=y+w*vertices(2,j+1);
      end
        plot(vertices(1,:),vertices(2,:),'b');
        hold on;grid on;
        axis tight;  
        xlabel('X');ylabel('Y');
        plot(x,y,'r');
    end

    if Dim==3
        x=(1-t).^(NumPoint)*vertices(1,1);
        y=(1-t).^(NumPoint)*vertices(2,1);
        z=(1-t).^(NumPoint)*vertices(3,1);
     for j=1:NumPoint
         w=factorial(NumPoint)/(factorial(j)*factorial(NumPoint-j))*(1-t).^(NumPoint-j).*t.^(j);
         x=x+w*vertices(1,j+1);y=y+w*vertices(2,j+1);z=z+w*vertices(3,j+1);
     end
        plot3(vertices(1,:),vertices(2,:),vertices(3,:),'b');
        hold on;grid on;
        axis tight;  
        %axis([0.5,1.5,0.5,1.5,0,0.7]);
        xlabel('X');ylabel('Y');zlabel('Z');
        plot3(x,y,z,'r');
        view(3);
    end

end


%% third order bezier curve 
% the points could be 3d position or 2d position
% p(t) = [p0, p1, p2, p3]*B*T
% B = [1, -3, 3, -1; 0, 3, -6, 3; 0, 0, 3, -3; 0, 0, 0,1];
% T = [1, t, t^2, t^3];