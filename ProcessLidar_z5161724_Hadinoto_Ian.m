function [xcoor,ycoor,xseg,yseg,xrob,yrob,xgf,ygf,xgfseg,ygfseg,linx,liny,linx2,liny2,linx3,liny3,linx4,liny4,linx5,liny5]=ProcessLidar_z5161724_Hadinoto_Ian(r,I,X,pt,theta,Mapx,Mapy);
    % Obtaining Cartesian coordinates from polar
    angleScan = ((0:360)/2)';
    xcoor = r.*cos(deg2rad(angleScan));
    ycoor = r.*sin(deg2rad(angleScan));
    
    % Segmenting OOIs
    i=1;
    k=1;
    segment = {};
    xseg = [];
    yseg = [];
    
    % Finding and isolating points on same poles
    while i<length(I)
        if I(i)>0
            j=1;
            while i<length(I) && I(i)>0 && sqrt((X(1,i+1)-X(1,i))^2+(X(2,i)-X(2,i+1))^2)<1
                temp(j) = i;
                i = i+1;
                j = j+1;
            end
            segment{k} = temp;
            temp = 0;
            k = k+1;
        end
        i = i+1;
    end
    
    % Taking the average of the points
    for i=1:length(segment)
        xseg(i) = mean(xcoor(segment{i}));
        yseg(i) = mean(ycoor(segment{i}));
    end    
    
    % Global Frame
    Rthetainv = [cos(theta(pt)) -sin(theta(pt));sin(theta(pt)) cos(theta(pt))];
    xgf = zeros(length(I),1);
    ygf = zeros(length(I),1);
    
    % Global Coordinates
    for i=1:length(I)
        Matrix = Rthetainv*[xcoor(i);ycoor(i)+0.46]+[X(1,pt);X(2,pt)];

        xgf(i) = Matrix(1);
        ygf(i) = Matrix(2);
    end
    
    % Global OOIs
    xgfseg = [];
    ygfseg = [];
    
    for i=1:length(segment)
        Matrix = Rthetainv*[xseg(i);yseg(i)+0.46]+[X(1,pt);X(2,pt)];

        xgfseg(i) = Matrix(1);
        ygfseg(i) = Matrix(2);
    end
    
    % Make robot
    xrob = zeros(9,1);
    yrob = zeros(9,1);
    robx = [0 -0.5 0.5 0 0 -10 10 0 0];
    roby = [0.5 -0.5 -0.5 0.5 0 0 0 0 0.5];
    
    % Robot angle shift
    for i=1:length(robx)
        Matrix = Rthetainv*[robx(i);roby(i)+0.46]+[X(1,pt);X(2,pt)];

        xrob(i) = Matrix(1);
        yrob(i) = Matrix(2);
    end
    
    linx = [];linx2 = [];linx3 = [];linx4 = [];linx5 = [];
    liny = [];liny2 = [];liny3 = [];liny4 = [];liny5 = [];
    
    % Plotting the mapped pole points 1
    for i=1:length(segment)
        if sqrt((Mapx(1)-xgfseg(i))^2+(Mapy(1)-ygfseg(i))^2)<0.6
            linx(1) = Mapx(1);
            linx(2) = xgfseg(i);
            liny(1) = Mapy(1);
            liny(2) = ygfseg(i);
        end
    end
    
    % Plotting the mapped pole points 2
    for i=1:length(segment)
        if sqrt((Mapx(2)-xgfseg(i))^2+(Mapy(2)-ygfseg(i))^2)<0.6
            linx2(1) = Mapx(2);
            linx2(2) = xgfseg(i);
            liny2(1) = Mapy(2);
            liny2(2) = ygfseg(i);
        end
    end
    
    % Plotting the mapped pole points 3
    for i=1:length(segment)
        if sqrt((Mapx(3)-xgfseg(i))^2+(Mapy(3)-ygfseg(i))^2)<0.6
            linx3(1) = Mapx(3);
            linx3(2) = xgfseg(i);
            liny3(1) = Mapy(3);
            liny3(2) = ygfseg(i);
        end
    end
    
    % Plotting the mapped pole points 4
    for i=1:length(segment)
        if sqrt((Mapx(4)-xgfseg(i))^2+(Mapy(4)-ygfseg(i))^2)<0.6
            linx4(1) = Mapx(4);
            linx4(2) = xgfseg(i);
            liny4(1) = Mapy(4);
            liny4(2) = ygfseg(i);
        end
    end
    
    % Plotting the mapped pole points 5
    for i=1:length(segment)
        if sqrt((Mapx(5)-xgfseg(i))^2+(Mapy(5)-ygfseg(i))^2)<0.6
            linx5(1) = Mapx(5);
            linx5(2) = xgfseg(i);
            liny5(1) = Mapy(5);
            liny5(2) = ygfseg(i);
        end
    end
    
    