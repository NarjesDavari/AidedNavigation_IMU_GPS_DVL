function [X,P] = kf_update(X,P,z,H,R)

       
%         S=H2*(M'*H2')*R*(H2*M)*H2' + H*P*H';
        S=R + H*P*H';
        K = P*H'/S;                 %Computed Kalman gain     
        P = P - K*H*P;              %Updated state covariance
        n = size(P,1);
%       P = (eye(n)-K*H)*P*(eye(n)-K*H)' + K*R*K';
        P = (P+P')/2;
        X = X + K * ( z-H*X);   %Updated state mean  

end