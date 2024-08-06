%LTI_DISC  Discretize LTI ODE with Gaussian Noise
%
% Syntax:
%   [A,Q] = lti_disc(E,F,Qc,dt)
%
% In:
%   E  - NxN Feedback matrix
%   F  - NxL Noise effect matrix        (optional, default identity)
%   Qc - LxL Diagonal Spectral Density  (optional, default zeros)
%   dt - Time Step                      (optional, default 1)
%
% Out:
%   A - Transition matrix
%   Q - Discrete Process Covariance
%
% Description:
%   Discretize LTI ODE with Gaussian Noise. The original
%   ODE model is in form
%
%     dx/dt = E x + F w,  w ~ N(0,Qc)
%
%   Result of discretization is the model
%
%     x[k] = A x[k-1] + q, q ~ N(0,Q)
%
%   Which can be used for integrating the model
%   exactly over time steps, which are multiples
%   of dt.
%Reference : my thesis page 57
function [A,Q] = lti_disc(F,G,Qc,dt)

  %
  % Closed form integration of transition matrix
  %
  n   = size(F,1);
  A=eye(n)+F*dt+(F*dt)*(F*dt)/2;
  
%   Q=G*Qc*G'*dt;
  Q=0.5*(A*G*Qc*G'*A'*dt+G*Qc*G'*dt);
  
  