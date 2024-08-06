function [Simulation]=Estimate_Q(Simulation)

Number_adaptive= Simulation.Output.Kalman_mtx.Number_adaptive;
update_counter=Simulation.Output.Kalman_mtx.update_counter;
Q=0;
% A=zeros(9,9,Number_adaptive+1);
% P=zeros(9,9,Number_adaptive+1);
% X_hat_plus=zeros(Number_adaptive+1,9);
% 
% A=Simulation.Output.Kalman_mtx.A_adap(:,:,update_counter-Number_adaptive:update_counter);
% P=Simulation.Output.Kalman_mtx.P_hat_plus(:,:,update_counter-Number_adaptive:update_counter);
% X_hat_plus=Simulation.Output.Kalman_mtx.X_hat_plus(update_counter-Number_adaptive:update_counter,:);

i=1;
     for j=update_counter-Number_adaptive+1:1:update_counter
         q(:,i)=Simulation.Output.Kalman_mtx.X_hat_minus(j,:)'-Simulation.Output.Kalman_mtx.A_adap(:,:,j)*Simulation.Output.Kalman_mtx.X_hat_plus(j-1,:)';   
         i=i+1;
     end
  
    mean_q(:,1) =1/Number_adaptive*sum(q,2);
   i=1;
   for k=update_counter-Number_adaptive:1:update_counter-1
       Q = Q+((q(:,i)-mean_q)*(q(:,i)-mean_q)');
%             -(Number_adaptive-1)/Number_adaptive*(Simulation.Output.Kalman_mtx.A_adap(:,:,k+1)*Simulation.Output.Kalman_mtx.P_hat_plus(:,:,k)*Simulation.Output.Kalman_mtx.A_adap(:,:,k+1)'-Simulation.Output.Kalman_mtx.P_hat_plus(:,:,k+1)));
       i=i+1;
   end
   
   Q_adap=1/(Number_adaptive-1)*Q;
%             -(Simulation.Output.Kalman_mtx.A_adap(:,:,k+1)*Simulation.Output.Kalman_mtx.P_hat_plus(:,:,k)*Simulation.Output.Kalman_mtx.A_adap(:,:,k+1)'-Simulation.Output.Kalman_mtx.P_hat_plus(:,:,k+1));

   Simulation.Output.Kalman_mtx.Q=Q_adap;
  
   