function [Xe,P]=DoUpdate_z5161724_Hadinoto_Ian(P,Xe,H,R,z)
    % Some intermediate steps for the EKF (as presented in the lecture notes)
    S = R + H*P*H' ;
    %iS=inv(S);                 % iS = inv(S) ;   % in this case S is 1x1 so inv(S) is just 1/S
    K = P*H'/S ;           % Kalman gain
    % ----- finally, we do it...We obtain  X(k+1|k+1) and P(k+1|k+1)

    Xe = Xe+K*z ;       % update the  expected value
    P = P-K*H*P ;       % update the Covariance % i.e. "P = P-P*H'*iS*H*P"  )
end 