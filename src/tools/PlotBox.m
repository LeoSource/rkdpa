function PlotBox(pos, size, style)
%%to d: plot oriented box
    switch style
        case 'center'
            idx=0;
            for i1=-1:2:1
                for i2=-1:2:1
                    for i3=-1:2:1
                        idx=idx+1;
                        point(idx,:) = [pos(1)+sign(i1)*0.5*size(1), pos(2)+sign(i2)*0.5*size(2), pos(3)+sign(i3)*0.5*size(3)];
                    end
                end
            end
        case 'corner'
            idx=0;
            for i1=0:1
                for i2=0:1
                    for i3=0:1
                        idx=idx+1;
                        point(idx,:) = [pos(1)+sign(i1)*size(1), pos(2)+sign(i2)*size(2), pos(3)+sign(i3)*size(3)];
                    end
                end
            end            
        otherwise
            error('please input the right plot style');
    end
    shp = alphaShape(point);
    plot(shp, 'edgecolor', 'none');
    xlabel('x');ylabel('y');zlabel('z');
    camlight

end