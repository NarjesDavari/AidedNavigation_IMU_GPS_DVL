function [dX,P] = eskf_update(dX,P,dz,H,H2,M,R,I)

       
%         S=H2*(M'*H2')*R*(H2*M)*H2' + H*P*H';
        S=R + H*P*H';
        K = P*H'/S;                 %Computed Kalman gain   
%         K2 = P*H'*(R^(-1)-R^(-1)*H*(eye(15)+P*H'*R^(-1)*H)^(-1)*P*H'*R^(-1));
%         P = P - K*H*P;              %Updated state covariance
        
        %%%%%%
%         P = P -K*(H*P*H'+R)*K';
        P = (eye(15)-K*H)*P*(eye(15)-K*H)' + K*R*K';
        %%%%%%
%         [u,d,v]=svd(P);
%         P=u*sqrt(d)*u';
        %%%%%%
        n = size(P,1);
%       P = (eye(n)-K*H)*P*(eye(n)-K*H)' + K*R*K';
        P = (P+P')/2;
        dX = dX + K * ( dz-H*dX);   %Updated state mean  

end