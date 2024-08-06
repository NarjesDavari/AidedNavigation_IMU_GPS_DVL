%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Variational Bayesian ( P and R adaptive estimation) update

function [Simulation,X,P]= VB_QR_Estimat(Simulation,X_,P_, Y ,H , I)
    
global GPS_fusion_active ;
global depth_fusion_active ;
global DVL_fusion_active ;
global Hdng_fusion_active ;
global accelrollpitch;

update_counter=Simulation.Output.Kalman_mtx.update_counter;
M=Simulation.Output.Kalman_mtx.M;
H1=Simulation.Output.Kalman_mtx.H1;
H2=Simulation.Output.Kalman_mtx.H2;

m= size(Y,1);
n=size(X_,2);
 tau= Simulation.parameter_VB_adaptiveQR.tau;
 rho = 1-exp(-Simulation.parameter_VB_adaptiveQR.rho);
 N = Simulation.parameter_VB_adaptiveQR.N;
% X_ =F*X;
u_hat_ = rho*(Simulation.Output.Kalman_mtx.VB_adaptiveQR.u_hat(update_counter,1)-m-1) + m + 1;%%Simulation.Output.Kalman_mtx.VB_adaptiveQR.u_hat(update_counter,1); %% 
C = sqrt(rho)*eye(length(Simulation.Output.Kalman_mtx.VB_adaptiveQR.U_hat(:,:,update_counter)));
U_hat_ = C*Simulation.Output.Kalman_mtx.VB_adaptiveQR.U_hat(:,:,update_counter)*C';
% P_=F*P*F'+Q;
T_hat_ = tau*P_; %%%sqrt(rho)*eye(length(P_))*P_*sqrt(rho)*eye(length(P_)); %%%
t_hat_ = n+tau+1;%%%%%%rho*(Simulation.Output.Kalman_mtx.VB_adaptiveQR.t_hat(update_counter,1)-n-1) + n + 1; %%%Simulation.Output.Kalman_mtx.VB_adaptiveQR.t_hat(update_counter,1);%% 

X=X_;
P=P_;

U_hat1=H1*U_hat_;
U_hat2=H2*U_hat_;
for i=1:N
    %%%%%%%%%%%%%%%% update q(P_)
    A = P + (X-X_)*(X-X_)';
    T_hat = 0.5*A + T_hat_;
    t_hat = t_hat_ +1;
    P_hat = T_hat /(t_hat - n -1);
    %%%%%%%%%%%%%%%% update q(R)
    B = (Y-H*X')*(Y-H*X')' + H*P*H';
    U_hat = 0.5*H(:,1:9)'*B*H(:,1:9) + U_hat_;
    u_hat  = u_hat_ +1;
    R_hat = U_hat /(u_hat - m -1);
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   S =H* P_hat*H' + H(:,1:9)*M*R_hat*M'*H(:,1:9)';
    [u,d,v1]=svd(S);
        S=u*sqrt(d)*u';
        K = P_hat*H'/diag(diag(S));  
%    P_hat =utchol(P_hat);
%    K=  P_hat * H'* inv(H* P_hat*H' + H(:,1:9)*M*R_hat*M'*H(:,1:9)');
   X = (X_' + K*(Y- H*X_'))';
   P = P_hat - K * H* P_hat;
end
 U_hat_f=diag(H1*diag(U_hat))+ U_hat2;
 
Simulation.Output.Kalman_mtx.VB_adaptiveQR.u_hat(Simulation.Output.Kalman_mtx.update_counter+1,1)=u_hat;
Simulation.Output.Kalman_mtx.VB_adaptiveQR.U_hat(:,:,Simulation.Output.Kalman_mtx.update_counter+1)=U_hat_f;
Simulation.Output.Kalman_mtx.VB_adaptiveQR.P_hat(:,:,Simulation.Output.Kalman_mtx.update_counter)=P_hat;   
Simulation.Output.Kalman_mtx.VB_adaptiveQR.t_hat(update_counter+1,1) = t_hat;
 
   if update_counter~=1
        if GPS_fusion_active
             Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter-1,1)=R_hat(1,1);
             Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter-1,2)=R_hat(2,2);
        end
        if depth_fusion_active
             Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter-1,1)=R_hat(3,3);
        end
        if DVL_fusion_active
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter-1,1)=R_hat(4,4);
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter-1,2)=R_hat(5,5);
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter-1,3)=R_hat(6,6);
        end
        if accelrollpitch
             Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter,1)=R_hat(7,7);
             Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter,2)=R_hat(8,8);
        end
        if Hdng_fusion_active
             Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter-1,1)=R_hat(9,9);
        end
    end