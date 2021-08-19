function [xcoor,ycoor,xseg,yseg,xrob,yrob,xgf,ygf,xgfseg,ygfseg,nDetectedLandmarks,Xe,Xdr,P,linx,liny,linx2,liny2,linx3,liny3,linx4,liny4,linx5,liny5]=ProcessEKF_z5161724_Hadinoto_Ian(dt,v,w,wgb,P,r,I,Xe,Xdr,pt,theta,Mapx,Mapy)
    % EKF Calculations
    % Standard deviation of the error in the angular rate sensor  (i.e. std of noise in gyro's measurements). 
    stdDevGyro = 1.7*pi/180 ;        %"actual"

    % Standard deviation of the error in the speed's measurements
    stdDevSpeed = 0.3 ;

    % ... errors in the range and angle measurements (0.35m, 2.5deg, standard dev.)
    sdev_rangeMeasurement = 0.35 ;
    sdev_angleMeasurement = 2.5;

    % -------- THIS IS THE ACTUAL EKF! ------------------------

    % ------ EKF's prediction: applies the prediction step (i.e. Process model). 
    % Estimate new covariance, associated to the state after prediction
    % First , I evaluate the Jacobian matrix of the process model (see lecture notes), at X=X(k|k).
    % You should write the analytical expression on paper to understand the following line.
    J = [[1,0,-dt*v(pt)*sin(Xe(3)),0];[0,1,dt*v(pt)*cos(Xe(3)),0];[0,0,1,-dt];[0,0,0,1]]; 
    % this J is explanined in "[AAS]_L05_Localization Problem Using EKF[2021].pdf"
    
    % Obtaining Q matrix
    Ju = [dt*cos(Xe(3)),0;dt*sin(Xe(3)),0;0,dt;0,0];
    Pu = diag([stdDevSpeed^2,stdDevGyro^2]);
    Qu = Ju*Pu*Ju';
    Q2 = diag([(dt*0.03)^2,(dt*0.03)^2,0,(dt*pi/180/10/60)^2]); % Check Q2 Error
    Q = Qu+Q2;

    % then I calculate the new covariance matrix, due to the prediction, P(K+1|K) = J*P(K|K)*J'+Q ;
    P = J*P*J'+Q ;
    % ATTENTION: we need, in our case, to propose a consistent Q matrix (this is part of your assignment!)
        
    % And, here, we calculate the predicted expected value. 
    Xe = RunProcessModel2_z5161724_Hadinoto_Ian(Xe,pt,v,w,dt);
    Xdr = RunProcessModel_z5161724_Hadinoto_Ian(Xdr,pt,v,wgb,dt);
    %  Xe(k+1|k) = f( Xe(k|k), u(k) )    % "Xe" means expected value.
    % in our case, we do not perfectly know the inputs u(k), but me measure them (which are noisy)
    % note: we reuse the same variable Xe (we do not need to remember Xe(k|k), so we overwrite it.)
    
    % so, here/now, the variable "Xe" contains "X^(k+1|k)"   ("X^" means "X hat", i.e. predicted expected value of X(k+1) )
    % and the variable "P" is "P(k+1|k)".
    
    % The prediction step, for this iteration, is done.
    [xcoor,ycoor,xseg,yseg,xrob,yrob,xgf,ygf,xgfseg,ygfseg,linx,liny,linx2,liny2,linx3,liny3,linx4,liny4,linx5,liny5]=ProcessLidar2_z5161724_Hadinoto_Ian(r,I,Xe,pt,theta,Mapx,Mapy);

    % .. Get range measuremens, if those are available.
    [dx,dy,nDetectedLandmarks,ExpectedRange,ExpectedBearing,MeasuredRange,MeasuredBearing] = GetMeasurementsFomNearbyLandmarks_z5161724_Hadinoto_Ian(Xe,linx,liny,linx2,liny2,linx3,liny3,linx4,liny4,linx5,liny5);
    
    % if measurements are avaiable ==> we perform the related updates.
    if nDetectedLandmarks>0     % any laser data and detected landmarks?
     
        % Because there are available obsevations ==> I perform EKF update\updates.
   
        % --------------- EKF update (observations)
        % Because this observation function is non-linear--> we need to get the Jacobians of h(X).
        % Jacobian for range only measurements (evaluated at the current expected value -->Xe)    
        
        % sqrt( (xi-x)^2+(yi-y)^2 ) for all the seen landmarks. I will need
        % this term for the next calculations.
        
        % for the sake of simplicity in this example, we perform an EKF update for each of the observations.
        for u=1:nDetectedLandmarks
            % here is it. "H". I reuse some previous calculations.
            H = [-dx(u)/ExpectedRange(u) -dy(u)/ExpectedRange(u) 0 0;dy(u)/ExpectedRange(u)^2 -dx(u)/ExpectedRange(u)^2 -1 0] ;   % Jacobian of h(X); size 2x3
             % see/read the lecture notes about EKF applied for implementing the localizer.   
        
            % Evaluate residual (innovation)  "Y-h(Xe)" 
            %(measured output value - expected output value)
            z  = [MeasuredRange(u)-ExpectedRange(u);wrapToPi(MeasuredBearing(u)-ExpectedBearing(u))] ;      

            % ------ covariance of the noise/uncetainty in the measurements
            R = diag([sdev_rangeMeasurement^2,sdev_angleMeasurement^2]);
            
            [Xe,P]=DoUpdate_z5161724_Hadinoto_Ian(P,Xe,H,R,z);
        
            % Loop to the next observation based on available measurements..
        end
       
       % Here, all available EKF updates have been done.
       % which means that the variable Xe contains X^(k+1|k+1); and P is P(k+1|k+1); the
       % expected value and covariance matrix, respectively, of the POSTERIOR PDF about X(k+1)
       
       
    end
    
    
    
    
    