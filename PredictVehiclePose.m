% X0 current state ( [x; y; heading] )
% X estimated next state ( [x; y; heading] at time t+dt)
% speed : current speed (m/s)
% steering : current steering angle (at time t)(in radians)
% dt is the "integration" horizon (should be a fraction of second)
% Jose Guivant - For AAS
% Tricycle / Ackermann model, discrete version
function X = PredictVehiclePose(X0,wt,speed,dt)
    % Remember: state vector X = [x; y; heading]
    X=X0;
    dL = dt*speed;
    X(3) = wt;
    X(1:2) = X0(1:2)+dL*[ cos(X0(3));sin(X0(3))] ;
return ;