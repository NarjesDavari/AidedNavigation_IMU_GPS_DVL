%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Variational Bayesian ( P and R adaptive estimation) update

function [Simulation,dX,P,u_hat,U_hat_f,P_hat,t_hat]= VB_QRestimat(Simulation,dX_,P_, dz ,H,M , I, update_counter)
    
global GPS_fusion_active ;
global depth_fusion_active ;
global DVL_fusion_active ;
global Hdng_fusion_active ;
global accelrollpitch;
% global Updt_Cntr;

H1=Simulation.Output.Kalman_mtx.H1;
H2=Simulation.Output.Kalman_mtx.H2;
m= size(dz,1);
n=size(dX_,1);
 tau= Simulation.parameter_VB_adaptiveQR.tau;
 rho = 1-exp(-Simulation.parameter_VB_adaptiveQR.rho);
 N = Simulation.parameter_VB_adaptiveQR.N;
% dX_ =F*dX;
u_hat_ = rho*(Simulation.Output.Kalman_mtx.VB_adaptiveQR.u_hat(update_counter,1)-m-1) + m + 1;
C = sqrt(rho)*eye(length(Simulation.Output.Kalman_mtx.VB_adaptiveQR.U_hat(:,:,update_counter)));
U_hat_ = C*Simulation.Output.Kalman_mtx.VB_adaptiveQR.U_hat(:,:,update_counter)*C';
% U_hat_ = rho*Simulation.Output.Kalman_mtx.VB_adaptiveQR.U_hat(:,:,update_counter);
% P_=F*P*F'+Q;
T_hat_ = tau*P_;
t_hat_ = n+tau+1;

dX=dX_;
P=P_;

U_hat1=H1*U_hat_;
U_hat2=H2*U_hat_;
for i=1:N
    %%%%%%%%%%%%%%%% update q(P_)
    A = P + (dX-dX_)*(dX-dX_)';
    T_hat = A + T_hat_;
    t_hat = t_hat_ +1;
   P_hat = T_hat /(t_hat - n -1);
    %%%%%%%%%%%%%%%% update q(R)
    B = (dz-H*dX)*(dz-H*dX)' + H*P*H';
    U_hat = 0.22*H(:,1:9)'*B*H(:,1:9) + U_hat_;
    u_hat  = u_hat_ +1;
     R_hat = U_hat /(u_hat - m -1);
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    P_hat =utchol(P_hat);
 S =H* P_hat*H' + H(:,1:9)*M*R_hat*M'*H(:,1:9)';
    [u,d,v1]=svd(S);
        S=u*sqrt(d)*u';
        K = P_hat*H'/diag(diag(S));  
%    K=  P_hat * H'* inv(H* P_hat*H' + H(:,1:9)*M*R_hat*M'*H(:,1:9)');
   dX = dX_ + K*(dz- H*dX_);
   P = P_hat - K * H* P_hat;
   
   %%%%%%%%%%%%%%%for checking inversion lemma
%     R2 = H(:,1:9)*M*R_hat*M'*H(:,1:9)';
%     S_inv = R2^(-1) - R2^(-1)*H * (P_hat^(-1)+H'*R2^(-1)*H)^(-1)*H'*R2^(-1);
% %     [u,d,v1]=svd(S_inv);
% %         S_inv=u*sqrt(d)*u';
%     K = P_hat*H'*diag(diag(S_inv));
%     dX = dX_ + K*(dz- H*dX_);
%    P = P_hat - K * H* P_hat;
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
end
 U_hat_f=diag(H1*diag(U_hat))+ U_hat2;
    
 
 
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