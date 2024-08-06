%EKF_UPDATE1  1st order Extended Kalman Filter update step
%
% Syntax:
%   [M,P,K,MU,S,LH] = EKF_UPDATE1(M,P,Y,H,R,[h,V,param])
%
% In:
%   M  - Nx1 mean state estimate after prediction step
%   P  - NxN state covariance after prediction step
%   Y  - Dx1 measurement vector.
%   H  - Derivative of h() with respect to state as matrix,
%        inline function, function handle or name
%        of function in form H(x,param)
%   R  - Measurement noise covariance.
%   h  - Mean prediction (innovation) as vector,
%        inline function, function handle or name
%        of function in form h(x,param).               (optional, default H(x)*X)
%   V  - Derivative of h() with respect to noise as matrix,
%        inline function, function handle or name
%        of function in form V(x,param).               (optional, default identity)
%   param - Parameters of h                            (optional, default empty)
%
% Out:
%   M  - Updated state mean
%   P  - Updated state covariance
%   K  - Computed Kalman gain
%   MU - Predictive mean of Y
%   S  - Predictive covariance of Y
%   LH - Predictive probability (likelihood) of measurement.
%   
% Description:
%   Extended Kalman Filter measurement update step.
%   EKF model is
%
%     y[k] = h(x[k],r),   r ~ N(0,R)

function [X,P] = ekf_update1(X_hat_minus , P_minus , Y , H , R,I)
 
  %
  % update step
  %  
  S = (R + H*P_minus*H');
  K = P_minus*H'/S;
  X = (X_hat_minus' + (K * (Y - H*X_hat_minus')))';
  P = P_minus - K * H * P_minus;

