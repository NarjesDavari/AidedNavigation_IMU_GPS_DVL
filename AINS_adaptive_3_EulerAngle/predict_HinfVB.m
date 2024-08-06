%KF_PREDICT  Perform Kalman Filter prediction step
%Reference: Strapdown inertial navigation system, chapter 13 page 406 
function [Simulation,dX,P,alpha,beta] = predict_HinfVB(Simulation,gama2,I,ave_sample)

    P=Simulation.Output.Kalman_mtx.P;
    A=Simulation.Output.Kalman_mtx.A;
    Q=Simulation.Output.Kalman_mtx.Q; 
%     G=Simulation.Output.Kalman_mtx.G;
    alpha=Simulation.Output.Kalman_mtx.alpha_VB;
    beta=Simulation.Output.Kalman_mtx.beta_VB;
    rho=1-exp(-Simulation.Output.Kalman_mtx.rho_VB);%%%%????
%     R = Simulation.Output.Kalman_mtx.R.Rmatrx; 
%     H = Simulation.Output.Kalman_mtx.H;
    dX = zeros(15,1);
%      dX = A*Simulation.Output.ESKF.dX(I - ave_sample,:)';

% if (~isempty (Simulation.Output.Kalman_mtx.H))
%         P  =( A * P *(eye(length(dX))-gama2* P + H'* diag(1./diag(R))* H *P)^(-1) * A' + Q); %Predicted state covariance  
% else
      P  =( A * P * A' + Q); %Predicted state covariance  
% end
%         [u,d,v1]=svd(P);
%         P=u*sqrt(d)*u';
        
        Simulation.Output.Kalman_mtx.P_diag(I-ave_sample+1,:)=[P(1,1) P(2,2) P(3,3) P(4,4) P(5,5) P(6,6) P(7,7) P(8,8) P(9,9) P(10,10) P(11,11) P(12,12) P(13,13) P(14,14) P(15,15)];
       alpha = rho'.*alpha;
       beta = rho'.*beta;
end
