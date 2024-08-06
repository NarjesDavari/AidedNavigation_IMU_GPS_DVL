%KF_PREDICT  Perform Kalman Filter prediction step
%Reference: Strapdown inertial navigation system, chapter 13 page 406 
function [Simulation,dX,P,alpha,beta] = eskf_predict(Simulation,I,ave_sample,Updt_Cntr)

    P=Simulation.Output.Kalman_mtx.P;
    A=Simulation.Output.Kalman_mtx.A;
    Q=Simulation.Output.Kalman_mtx.Q; 
    G=Simulation.Output.Kalman_mtx.G;
    alpha=Simulation.Output.Kalman_mtx.alpha_VB;
    beta=Simulation.Output.Kalman_mtx.beta_VB;
    rho=1-exp(-Simulation.Output.Kalman_mtx.rho_VB);%%%%????
    

     gama2 =Simulation.parameter_HinfVB.gama2;
   
     dX = zeros(15,1);
%      dX = A*Simulation.Output.ESKF.dX(I - ave_sample,:)';
% if Updt_Cntr ==0 || (isempty (Simulation.Output.Kalman_mtx.H))
         P  =( A * P * A' + Q); %Predicted state covariance on ESKF   
% else
%      R = Simulation.Output.Kalman_mtx.R.Rmatrx; 
%      H = Simulation.Output.Kalman_mtx.H;
%      eta = (eye(length(dX))-gama2* P + H'* R^(-1)* H *P)^(-1);
%       P  = A * P *eta * A' + Q; %Predicted state covariance on H-infinity
% end
%  
%   [u,d,v]=svd(P);
%    P=u*sqrt(d)*u';

Simulation.Output.Kalman_mtx.P_diag(I-ave_sample+1,:)=[P(1,1) P(2,2) P(3,3) P(4,4) P(5,5) P(6,6) P(7,7) P(8,8) P(9,9) P(10,10) P(11,11) P(12,12) P(13,13) P(14,14) P(15,15)];
       alpha = rho'.*alpha;
       beta = rho'.*beta;
end
