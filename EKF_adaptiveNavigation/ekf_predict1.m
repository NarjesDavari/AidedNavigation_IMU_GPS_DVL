%EKF_PREDICT1  1st order Extended Kalman Filter prediction step
%
% Syntax:
%   [M,P] = EKF_PREDICT1(M,P,[A,Q,a,W,param])
%
% In:
%   M - Nx1 mean state estimate of previous step
%   P - NxN state covariance of previous step
%   A - Derivative of a() with respect to state as
%       matrix, inline function, function handle or
%       name of function in form A(x,param)       (optional, default eye())
%   Q - Process noise of discrete model               (optional, default zero)
%   a - Mean prediction E[a(x[k-1],q=0)] as vector,
%       inline function, function handle or name
%       of function in form a(x,param)                (optional, default A(x)*X)
%   W - Derivative of a() with respect to noise q
%       as matrix, inline function, function handle
%       or name of function in form W(x,param)        (optional, default identity)
%   param - Parameters of a                           (optional, default empty)
%
% Out:
%   M - Updated state mean
%   P - Updated state covariance
%   
% Description:
%   Perform Extended Kalman Filter prediction step.

function [X_hat_minus , P_minus , fnn , Wnb_b] = ekf_predict1(X_ , P_ , func_f , A_ , Q_ , C_ , fb , fb_ , Wib_b , Wib_b_ , gl , dt)


  %
  % Perform prediction
  %
    % Rectangular Integration
    [delta_X , fnn , Wnb_b] = feval(func_f , X_ , C_ , fb_ , Wib_b_ , gl);
    X_hat_minus = X_ + delta_X' * dt;
    
% %     % Trapezoidal Integration
%     [X1_dot , fnn1 , Wnb_b1]           = feval(func_f , X_ , C_  , fb_ , Wib_b_ , gl);
%     delta_X1                         =  X1_dot * dt;
%     X_plus_delta_X1                  = X_ + delta_X1';
%     Euler_I                          = [X_plus_delta_X1(7),X_plus_delta_X1(8),X_plus_delta_X1(9)] ;
%     Cbn                              = InCBN(Euler_I);
%     [X2_dot , fnn2 , Wnb_b2]           = feval(func_f , X_plus_delta_X1 , Cbn  , fb  , Wib_b  , gl);
%     delta_X2                         =  X2_dot * dt;
%     delta_X                          = (delta_X1 + delta_X2)/2;
%     X_dot                            = (X1_dot + X2_dot)/2;
%     fnn                              = (fnn1+fnn2)/2;
%     Wnb_b                            = (Wnb_b1+Wnb_b2)/2;
%     X_hat_minus                      = X_ + delta_X';
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Euler Range Checking
    if X_hat_minus(9) > pi
        X_hat_minus(9) = X_hat_minus(9) - 2*pi;
    elseif X_hat_minus(9) < -pi
        X_hat_minus(9) = X_hat_minus(9) + 2*pi;
    else
        %No Operation
    end    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    P_minus = A_ * P_ * A_' + Q_;    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
