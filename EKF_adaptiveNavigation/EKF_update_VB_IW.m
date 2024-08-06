%KF_UPDATE, Kalman Filter update step
%Reference: Strapdown inertial navigation system, chapter 13 page 406 
function [Simulation,X,P,alpha,beta] = EKF_update_VB_IW(Simulation,X_,P_,Y,H,alpha_,beta_,I)

update_counter=Simulation.Output.Kalman_mtx.update_counter;
%     A=Simulation.Output.Kalman_mtx.A;
%     Qc=Simulation.Output.Kalman_mtx.QQc;
%     G=Simulation.Output.Kalman_mtx.G;
    M=Simulation.Output.Kalman_mtx.M;
    H1=Simulation.Output.Kalman_mtx.H1;
    H2=Simulation.Output.Kalman_mtx.H2;
    
global GPS_fusion_active ;
global depth_fusion_active ;
global DVL_fusion_active ;
global Hdng_fusion_active ;
global accelrollpitch;

N=Simulation.parameter_VB.N;
% c=(A*P_*A'+G*Qc*G');
rho=1-exp(-Simulation.Output.Kalman_mtx.rho_VB);
d_IW=length(nonzeros(Y));
alpha =alpha_+(1-rho)*(d_IW-1)+1; %*eye(length(alpha0));
beta1=H1*beta_;
beta2=H2*beta_;

    for i=1:N
        if update_counter==1
            R=Simulation.Output.Kalman_mtx.R.Rmatrx;
            S=H(:,1:9)*(M')*H(:,1:9)'*R*H(:,1:9)*(M)*H(:,1:9)' + H*P_*H';
        else
            R=diag(beta1./alpha);
            S=H(:,1:9)*M*R*M'*H(:,1:9)' + H*P_*H';
        end   
%         [u,d,v1]=svd(S);
%         S=u*sqrt(d)*u';
        S = utchol (S);
        K = P_*H'/diag(diag(S));   %Computed Kalman gain
        [m,n] = size(H);
        P = K * (H(:,1:9)*M*R*M'*H(:,1:9)') * K' + (eye(n)-K*H)*P_*(eye(n)-K*H)';
%         P = P_ - K*H*P_;              %Updated state covariance
%         n = size(P,1);
        %       P = (eye(n)-K*H)*P*(eye(n)-K*H)' + K*R*K';
        P = (P+P')/2;
        X = X_' + K * ( Y-H*X_');   %Updated state mean

        P_VB=P;
        X_VB=X;
        v=Y-H*X_VB;
        beta1 = H1*(0.5*beta_ + 0.5*0.5*H(:,1:9)'*(v.^2 + diag(H*P_VB*H')))/i;
%          beta1 = H1*(beta_ + 0.5*H(:,1:9)'*(v.^2 + diag(H*P_VB*H')));
    end
    beta=beta1+beta2;
    R=abs(diag(beta./alpha_));
    
    
    if update_counter~=1
        if GPS_fusion_active
             Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter,1)=R(1,1);
             Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter,2)=R(2,2);
        end
        if depth_fusion_active
             Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter,1)=R(3,3);
        end
        if DVL_fusion_active
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter,1)=R(4,4);
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter,2)=R(5,5);
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter,3)=R(6,6);
        end
        if accelrollpitch
             Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter,1)=R(7,7);
             Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter,2)=R(8,8);
        end
        if Hdng_fusion_active
             Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter,1)=R(9,9);
        end
    end
   
end