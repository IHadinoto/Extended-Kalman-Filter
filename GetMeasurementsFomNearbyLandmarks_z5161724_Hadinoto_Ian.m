function [dx,dy,nDetectedLandmarks,ExpectedRange,ExpectedBearing,MeasuredRange,MeasuredBearing] = GetMeasurementsFomNearbyLandmarks_z5161724_Hadinoto_Ian(X,linx,liny,linx2,liny2,linx3,liny3,linx4,liny4,linx5,liny5)
    if isempty(linx)
        linx=[0 0];
    end
    if isempty(linx2)
        linx2=[0 0];
    end
    if isempty(linx3)
        linx3=[0 0];
    end
    if isempty(linx4)
        linx4=[0 0];
    end
    if isempty(linx5)
        linx5=[0 0];
    end
    
    if isempty(liny)
        liny=[0 0];
    end
    if isempty(liny2)
        liny2=[0 0];
    end
    if isempty(liny3)
        liny3=[0 0];
    end
    if isempty(liny4)
        liny4=[0 0];
    end
    if isempty(liny5)
        liny5=[0 0];
    end
    
    % Map is (1), OOI is (2)
    dx = [linx(1)-X(1) linx2(1)-X(1) linx3(1)-X(1) linx4(1)-X(1) linx5(1)-X(1)];
    dy = [liny(1)-X(1) liny2(1)-X(1) liny3(1)-X(1) liny4(1)-X(1) liny5(1)-X(1)];
    dx = dx(dx~=0);
    dy = dy(dy~=0);
    
    % Map is (1), OOI is (2)
    ExpectedRange = [sqrt((linx(1)-X(1))^2+(liny(1)-X(2))^2) sqrt((linx2(1)-X(1))^2+(liny2(1)-X(2))^2) sqrt((linx3(1)-X(1))^2+(liny3(1)-X(2))^2) sqrt((linx4(1)-X(1))^2+(liny4(1)-X(2))^2) sqrt((linx5(1)-X(1))^2+(liny5(1)-X(2))^2)];
    ExpectedBearing = [atan2(liny(1)-X(2),linx(1)-X(1))-X(3)+pi/2 atan2(liny2(1)-X(2),linx2(1)-X(1))-X(3)+pi/2 atan2(liny3(1)-X(2),linx3(1)-X(1))-X(3)+pi/2 atan2(liny4(1)-X(2),linx4(1)-X(1))-X(3)+pi/2 atan2(liny5(1)-X(2),linx5(1)-X(1))-X(3)+pi/2];
    MeasuredRange = [sqrt((linx(2)-X(1))^2+(liny(2)-X(2))^2) sqrt((linx2(2)-X(1))^2+(liny2(2)-X(2))^2) sqrt((linx3(2)-X(1))^2+(liny3(2)-X(2))^2) sqrt((linx4(2)-X(1))^2+(liny4(2)-X(2))^2) sqrt((linx5(2)-X(1))^2+(liny5(2)-X(2))^2)];
    MeasuredBearing = [atan2(liny(2)-X(2),linx(2)-X(1))-X(3)+pi/2 atan2(liny2(2)-X(2),linx2(2)-X(1))-X(3)+pi/2 atan2(liny3(2)-X(2),linx3(2)-X(1))-X(3)+pi/2 atan2(liny4(2)-X(2),linx4(2)-X(1))-X(3)+pi/2 atan2(liny5(2)-X(2),linx5(2)-X(1))-X(3)+pi/2];
    % I simulate I measure/detect all the landmarks, however there can be
    % cases where I see just the nearby ones.
    ExpectedRange = ExpectedRange(MeasuredRange~=0);
    ExpectedBearing = ExpectedBearing(MeasuredBearing~=0);
    MeasuredRange = MeasuredRange(MeasuredRange~=0);
    MeasuredBearing = MeasuredBearing(MeasuredBearing~=0);
    nDetectedLandmarks = length(ExpectedRange);
end