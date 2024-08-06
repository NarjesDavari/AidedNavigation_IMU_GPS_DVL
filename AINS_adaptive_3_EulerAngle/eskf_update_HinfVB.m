%KF_UPDATE, Kalman Filter update step
%Reference: Strapdown inertial navigation system, chapter 13 page 406 
function [Simulation,dX,P,alpha,beta] = eskf_update_HinfVB(Simulation,dX_,P_,dz,H,M,I,alpha_,beta_,gama2)
update_counter=Simulation.Output.Kalman_mtx.update_counter;
%     A=Simulation.Output.Kalman_mtx.A;
%     Qc=Simulation.Output.Kalman_mtx.QQc;
%     G=Simulation.Output.Kalman_mtx.G;
    H1=Simulation.Output.Kalman_mtx.H1;
    H2=Simulation.Output.Kalman_mtx.H2;
    
global GPS_fusion_active ;
global depth_fusion_active ;
global DVL_fusion_active ;
global Hdng_fusion_active ;
global accelrollpitch
global Updt_Cntr;
N=Simulation.parameter_VB.N;
% c=(A*P_*A'+G*Qc*G');
alpha =(alpha_+0.5); %*eye(length(alpha0));
beta1=H1*beta_;
beta2=H2*beta_;

    for i=1:N
        if Updt_Cntr==1
            R=Simulation.Output.Kalman_mtx.R.Rmatrx;
            S=H(:,1:9)*(M')*H(:,1:9)'*R*H(:,1:9)*(M)*H(:,1:9)' + H*P_*H';
            R2=H(:,1:9)*(M')*H(:,1:9)'*R*H(:,1:9)*(M)*H(:,1:9)' ;
        else
            R=diag(beta1./alpha);
            S=H(:,1:9)*M*R*M'*H(:,1:9)' + H*P_*H';
            R2 = H(:,1:9)*M*R*M'*H(:,1:9)';  %%%diag(nonzeros(beta1)./alpha(1:length(nonzeros(beta1))));
        end   
%         

%%%%%% Calculation of inverse R
%             if GPS_fusion_active && ~depth_fusion_active && ~DVL_fusion_active
%                 R2_inv = diag(1./diag(R2));
%             elseif ~GPS_fusion_active && depth_fusion_active && ~DVL_fusion_active
%                 R2_inv = R2^(-1);
%             elseif ~GPS_fusion_active && ~depth_fusion_active && DVL_fusion_active
%                 R2_inv = R2^(-1);
%             elseif GPS_fusion_active && depth_fusion_active && ~DVL_fusion_active 
%                 R2_inv = diag(1./diag(R2));
%             elseif GPS_fusion_active && ~depth_fusion_active && DVL_fusion_active 
%                 R_GPS_inv =[1/R2(1,1) 0; 0 1/R2(2,2)];
%                 R_DVL=[R2(3,3) R2(3,4) R2(3,5);R2(4,3) R2(4,4) R2(4,5);R2(5,3) R2(5,4) R2(5,5)];
%                 R_DVL_inv = R_DVL^(-1); %%inv(R_DVL);
% %                 R2_inv = zeros(5);
%                 R2_inv = [R_GPS_inv zeros(2,3); zeros(3,2) R_DVL_inv];
%             elseif ~GPS_fusion_active && depth_fusion_active && DVL_fusion_active 
%                 R_depth_inv =1/R2(1,1);
%                 R_DVL= [R2(2,2) R2(2,3) R2(2,4);R2(3,2) R2(3,3) R2(3,4);R2(4,2) R2(4,3) R2(4,4)];
%                 R_DVL_inv =R_DVL^(-1);  %%inv(R_DVL);
%                 R2_inv = [R_depth_inv zeros(1,3);zeros(3,1) R_DVL_inv];
%             end
%%%%%%%
        R2_inv = (R2)^(-1);
%         R2 = diag(nonzeros(beta1)./alpha(1:length(nonzeros(beta1))));
        eta = (eye(length(dX_)) - gama2 * P_ + H'*R2_inv* H*P_)^(-1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Matrix Inversion Lemma
        
%         A_inv = (eye(length(dX_)) - gama2 * P_)^(-1);
%         %%%%% (inv(R2)
%         B = (R2_inv + H* P_ *A_inv * H');
% %         [a,b] = eig(B);
% %         B_inv = a*diag(1./diag(b))*a';
% %         eta = A_inv - A_inv * H' * B_inv * H * P_ * A_inv;
% %         eta = A_inv - A_inv * H' * B^(-1) * H * P_ * A_inv;
%         eta = A_inv - A_inv * H' *  B^(-1) * H * P_ * A_inv;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           [u,d,v1]=svd(eta);
%         eta=u*sqrt(d)*u';
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
%         K = P_*eta*H'*diag(1./diag(R2));                 %Computed Kalman gain
        K = P_ * eta*H'*R2_inv;  %%% diag(diag(eta))
%         P = P_ - K*H*P_;              %Updated state covariance
%       P = P -K*(H*P*H'+R)*K';
%        P = P_ * eta;
       P = P_ * eta;
%         [u,d,v1]=svd(P);
%         P=u*sqrt(d)*u';
       dX = dX_ + K * ( dz-H*dX_);   %Updated state mean
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%        P = (eye(15)-K*H)*P_*(eye(15)-K*H)' + K*R2*K';
% %         [u,d,v]=svd(P);
% %         P=u*sqrt(d)*u';
%         %       P = (eye(n)-K*H)*P*(eye(n)-K*H)' + K*R*K';
        P = (P+P')/2;% Force P to be symmetric.
%         lambda = eig(P);
%       if (abs(lambda(1)) >= 1) || (abs(lambda(2)) >= 1)
%          disp('gamma is too large');
% %          return;
%       end
%         dX = dX_ + K * ( dz-H*dX_);   %Updated state mean
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% chapter 12 from Dan Simon
%         a1= (eye(15)/gama2^2-P_)^(-1);
%         Pa = A*P_*H' + A*P_ *a1*P_*H';
%         V = (R2+ H*P_*H'+ H*P_*a1*P_*H');
%         K2 = Pa*V^(-1);
%         P = (eye(15)-K2*H)*P_*(eye(15)-K2*H)' + K2*R2*K2';
%         P = (P+P')/2;% Force P to be symmetric.
%         F_hat = A-K2*H;
%         dX = F_hat *dX_ + K2*(dz-H*dX_);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        P_VB=P;
        dX_VB=dX;
        v=dz-H*dX_VB;
%         beta1 = H1*(0.5*beta_ + 0.5*0.5*H(:,1:9)'*(v.^2 + diag(H*P_VB*H')))/i;
         beta1 = H1*(beta_ + 0.5*H(:,1:9)'*(v.^2 + diag(H*P_VB*H')));
    end
    beta=beta1+beta2;
    R=abs(diag(beta./alpha));
    Simulation.Output.Kalman_mtx.R.Rmatrx=diag(nonzeros(beta1)./alpha(1:length(nonzeros(beta1)))); 
    
    if Updt_Cntr~=1
        if GPS_fusion_active
             Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter-1,1)=R(1,1);
             Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter-1,2)=R(2,2);
        end
        if depth_fusion_active
             Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter-1,1)=R(3,3);
        end
        if DVL_fusion_active
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter-1,1)=R(4,4);
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter-1,2)=R(5,5);
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter-1,3)=R(6,6);
        end
        if accelrollpitch
             Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter,1)=R(7,7);
             Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter,2)=R(8,8);
        end
        if Hdng_fusion_active
             Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter-1,1)=R(9,9);
        end
    end
   
end