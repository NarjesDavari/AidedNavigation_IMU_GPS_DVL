%KF_UPDATE, Kalman Filter update step
%Reference: Strapdown inertial navigation system, chapter 13 page 406 
function [dX,P] = eskf_update_Hinf(A,Q,dX,P,dz,H,H2,M,R,gama2,I)
       
       eta = (eye(length(dX)) - gama2 * P + H'*R^(-1)* H*P)^(-1);
        K = P*eta*H'*R^(-1);    %Computed Kalman gain            
%         P = P - K*H*P;              %Updated state covariance
%          P = P -K*(H*P*H'+R)*K';
%         P = (eye(15)-K*H)*P*(eye(15)-K*H)' + K*R*K';
        
%         [u,d,v]=svd(P);
%         P=u*sqrt(d)*u';
        

%       P = (eye(n)-K*H)*P*(eye(n)-K*H)' + K*R*K';
%         P = (P+P')/2;
        dX = dX + K * (dz-H*dX);   %Updated state mean  
        P = P * eta; 
%         [u,d,v]=svd(P);
%         P=u*sqrt(d)*u';
        %%%%%%%%%%%%%%%%%%%%% chapter 12 from Dan Simon page 3
%         a1= (eye(15)/gama2^2-P)^(-1);
%         Pa = A*P*H' + A*P *a1*P*H';
%         V = (R+ H*P*H'+ H*P*a1*P*H');
%         K2 = Pa*V;
%         P = A* P* A' + Q + A*P * a1*P*A' - Pa*V^(-1)*Pa';
%         P = (P+P')/2;
%         F_hat = A-K2*H;
%         dX = F_hat *dX + K2*(dz-H*dX);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
%         cond = inv(P)-gama2 + H'*R^(-1)*H;
%         if eig(cond) < 0
%             disp('condition is not satisfied');
%         end
      lambda = eig(P);
      if (abs(lambda(1)) >= 1) || (abs(lambda(2)) >= 1)
         disp('gamma is too large');
         return;
      end
    
end