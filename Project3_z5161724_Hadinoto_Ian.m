close all;

%% Part A

Data=load('Measurements_AAS01.mat');       %We load the data
Data=Data.A;
Map=load('MapMTRN4010.mat');
Map=Map.Map;
Mapx=(Map.x)';
Mapy=(Map.y)';
map(:,1) = Mapx;
map(:,2) = Mapy;

t=Data.t; % sample times, {t}.     
t=double(t)*0.0001; %scale time to seconds; original data is integer type, 1 count = 0.1 ms.

% Scale speed to m/s ; originally in mm/s, integer type.
% scale heading rate to degrees/s ; originally in integer type, [1 count = 1/100 degrees/sec].
w = double(Data.Z(2,:))/100;                    % GyroZ readings, "{w(t)}"
gb = mean(w(1:4256)); % Gyro Bias
wgb = zeros(length(w),1);

% Gyro bias
for i=1:length(w)
    wgb(i) = w(i)-gb;
end
v = double(Data.Z(1,:))/1000;                     % speed readings   "{v(t)}" 
    
% plot those measurements.
figure;
subplot(211) ; plot(t,v);  xlabel('time (seconds)'); ylabel('speed (m/s)');
subplot(212) ; plot(t,wgb); xlabel('time (seconds)'); ylabel('angular rate (degrees/s)');

% Starting Values for robot going to the right
X = [0;0;0] ;  % initial pose. You will need it.

for k = 1:length(v)-1
    dt = t(k+1)-t(k);
    X(:,k+1) = RunProcessModel_z5161724_Hadinoto_Ian(X(:,k),k,v,wgb,dt);
end
theta = X(3,:); % save the value of theta

% Starting Values for robot going straight
X = [0;0;pi/2];  % initial pose. You will need it.

for k = 1:length(v)-1
    dt = t(k+1)-t(k);
    X(:,k+1) = RunProcessModel_z5161724_Hadinoto_Ian(X(:,k),k,v,wgb,dt);
end    

figure;plot(X(1,:),X(2,:));
grid on
xlabel('x axis')
ylabel('y axis')
title('Path of Robot using Model')
legend('Path')

%% Parts B,C,D,E
% Plotting Global Frame
figure;subplot(1,2,1);hold on;grid on;axis([-15,15,-10,10]);
hG=plot(0,0,'.');
hG2=plot(0,0,'.r');  % for showing brillant points, later.
hG3=plot(0,0,'or');  % for showing poles
hG4=plot(0,0,'-g');  % for showing robot
hM=plot(Mapx,Mapy,'sr');  % for showing Map pole points
hE=plot(0,0,'-r'); % for showing EKF estimator 1
hE2=plot(0,0,'-r'); % for showing EKF estimator 2
hE3=plot(0,0,'-r'); % for showing EKF estimator 3
hE4=plot(0,0,'-r'); % for showing EKF estimator 4
hE5=plot(0,0,'-r'); % for showing EKF estimator 5
legend({'Points','Brilliant Points','Poles','Robot','Actual Pole Positions'},'location','southeast');
title('Dead Reckoning');
xlabel('X distance');ylabel('Y distance'); 

% Plotting EKF Frame
subplot(1,2,2);hold on;grid on;axis([-15,15,-10,10]);
hEKF=plot(0,0,'.');
hEKF2=plot(0,0,'.r');  % for showing brillant points, later.
hEKF3=plot(0,0,'or');  % for showing poles
hEKF4=plot(0,0,'-g');  % for showing robot
hMM=plot(Mapx,Mapy,'sr');  % for showing Map pole points
hEE=plot(0,0,'-r'); % for showing EKF estimator 1
hEE2=plot(0,0,'-r'); % for showing EKF estimator 2
hEE3=plot(0,0,'-r'); % for showing EKF estimator 3
hEE4=plot(0,0,'-r'); % for showing EKF estimator 4
hEE5=plot(0,0,'-r'); % for showing EKF estimator 5
legend({'Points','Brilliant Points','Poles','Robot','Actual Pole Positions'},'location','southeast');
title('EKF');
xlabel('X distance');ylabel('Y distance');

L=Data.L;       %number of samples in this dataset.
i0=1; % you should start at i0=1 or close. (up to 52870)
P = diag([0,0,0,(2*pi/180)^2]);
Xe = [0;0;pi/2;0];
Xdr = [0;0;pi/2];
Xe_History = zeros(4,length(L)-1);
Xdr_History = zeros(3,length(L)-1);
disp('Running Code... Please Wait');
j=1;

%extract ranges and intensities, of the 361 "pixels" to initialise r
[r,I]=GetRangeAndIntensityFromRawScan_z5161724_Hadinoto_Ian(Data.scans(:,2));
ii=find(I>0);

for i=i0:L-1
    %tic
    m = Data.Z(:,i);
    indexScan = m(3);
    dt = t(i+1)-t(i);
    % Now, if there is a LiDAR scan at this time?
    if (indexScan>1)

        %extract ranges and intensities, of the 361 "pixels"
        [r,I]=GetRangeAndIntensityFromRawScan_z5161724_Hadinoto_Ian(Data.scans(:,indexScan));

        [~,~,~,~,xrob,yrob,xgf,ygf,xgfseg,ygfseg,linx,liny,linx2,liny2,linx3,liny3,linx4,liny4,linx5,liny5]=ProcessLidar_z5161724_Hadinoto_Ian(r,I,X,i,theta,Mapx,Mapy);
        %here you may process the LiDAR
        %data.  variable "X" is the current pose of the platform
        %(assuming you keep estimating it). If your procesing of the
        %LiDAR data does require it.    
        % if we want to show Global Frame, ...
        set(hG,'xdata',xgf,'ydata',ygf);
        % which points do have intensity>0??
        ii=find(I>0);set(hG2,'xdata',xgf(ii),'ydata',ygf(ii));
        % Grouping brilliant points
        set(hG3,'xdata',xgfseg,'ydata',ygfseg);
        % Showing Robot moving
        set(hG4,'xdata',xrob,'ydata',yrob);
        % Showing linear mapping
        set(hE,'xdata',linx,'ydata',liny);
        set(hE2,'xdata',linx2,'ydata',liny2);
        set(hE3,'xdata',linx3,'ydata',liny3);
        set(hE4,'xdata',linx4,'ydata',liny4);
        set(hE5,'xdata',linx5,'ydata',liny5);
    end
    
    % EKF
    [xcoor,ycoor,xseg,yseg,xrob,yrob,xgf,ygf,xgfseg,ygfseg,nDetectedLandmarks,Xe,Xdr,P,linx,liny,linx2,liny2,linx3,liny3,linx4,liny4,linx5,liny5]=ProcessEKF_z5161724_Hadinoto_Ian(dt,v,w,wgb,P,r,I,Xe,Xdr,i,theta,Mapx,Mapy);
    Xe_History(:,i)=Xe;
    Xdr_History(:,i)=Xdr;
    
     % if we want to show EKF Frame, ...
    set(hEKF,'xdata',xgf,'ydata',ygf);
    % which points do have intensity>0??
    ii=find(I>0);set(hEKF2,'xdata',xgf(ii),'ydata',ygf(ii));
    % Grouping brilliant points
    set(hEKF3,'xdata',xgfseg,'ydata',ygfseg);
    % Showing Robot moving
    set(hEKF4,'xdata',xrob,'ydata',yrob);
    % Showing linear mapping
    set(hE,'xdata',linx,'ydata',liny);
    set(hE2,'xdata',linx2,'ydata',liny2);
    set(hE3,'xdata',linx3,'ydata',liny3);
    set(hE4,'xdata',linx4,'ydata',liny4);
    set(hE5,'xdata',linx5,'ydata',liny5);
    %pause(0.001);  % short pause, just to see some animation, more slowly.You may change this.
    %toc
end

disp('Done. Showing results now...');
figure; hold on;
plot(Xdr_History(1,:),Xdr_History(2,:),'r');
plot(Xe_History(1,:),Xe_History(2,:),'b');
plot(Mapx,Mapy,'+r','markersize',15,'linewidth',3) ;
legend({'DR Estimated path','EKF Estimated path','Landmarks'});

figure;hold on;grid on;
plot(t(1:end-1),rad2deg(Xe_History(4,:)),'r');
plot([0,t(end-1)],[gb gb],'b');
legend('Gyro Bias EKF','Gyro Bias Stationary');
title('Gyro Bias');
xlabel('Time/s');ylabel('Bias/degrees s^-^1');
%-------------------------------------------------------
