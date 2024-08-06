%KF_UPDATE, Kalman Filter update step
%Reference: Strapdown inertial navigation system, chapter 13 page 406 
function [Simulation,dX,P,alpha,beta] = eskf_update_VB_2(Simulation,dX_,P_,dz,H,M,I,alpha_,beta_)
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
            R2=R;
            S_inv = inv(S);
        else
            R=diag(beta1./alpha);
            S2=H(:,1:9)*M*R*M'*H(:,1:9)' + H*P_*H';
            %%%%%%%%%%
            R2 = H(:,1:9)*M*R*M'*H(:,1:9)';
           
            if GPS_fusion_active && ~depth_fusion_active && ~DVL_fusion_active
                R2_inv = diag(1./diag(R2));
            elseif ~GPS_fusion_active && depth_fusion_active && ~DVL_fusion_active
                R2_inv = R2^(-1);
            elseif ~GPS_fusion_active && ~depth_fusion_active && DVL_fusion_active
                R2_inv = R2^(-1);
            elseif GPS_fusion_active && depth_fusion_active && ~DVL_fusion_active 
                R2_inv = diag(1./diag(R2));
            elseif GPS_fusion_active && ~depth_fusion_active && DVL_fusion_active 
                R_GPS_inv =[1/R2(1,1) 0; 0 1/R2(2,2)];
                R_DVL=[R2(3,3) R2(3,4) R2(3,5);R2(4,3) R2(4,4) R2(4,5);R2(5,3) R2(5,4) R2(5,5)];
                R_DVL_inv = R_DVL^(-1); %%inv(R_DVL);
%                 R2_inv = zeros(5);
                R2_inv = [R_GPS_inv zeros(2,3); zeros(3,2) R_DVL_inv];
            elseif ~GPS_fusion_active && depth_fusion_active && DVL_fusion_active 
                R_depth_inv =1/R2(1,1);
                R_DVL= [R2(2,2) R2(2,3) R2(2,4);R2(3,2) R2(3,3) R2(3,4);R2(4,2) R2(4,3) R2(4,4)];
                R_DVL_inv =R_DVL^(-1);  %%inv(R_DVL);
                R2_inv = [R_depth_inv zeros(1,3);zeros(3,1) R_DVL_inv];
            end
             
     
            S_inv = R2_inv-R2_inv*H*(inv(P_)+H'*R2_inv*H)^(-1)*H'*R2_inv;
           %%%%%%%% 
        end   
%         [u,d,v1]=svd(S);
%         S=u*sqrt(d)*u';
              
        
        K = P_*H'*S_inv;   %%diag(diag(S_inv))  %Computed Kalman gain
%         P = P_ - K*H*P_;              %Updated state covariance
        P = (eye(15)-K*H)*P_ *(eye(15)-K*H)'+ K*R2*K';
        n = size(P,1);
        %       P = (eye(n)-K*H)*P*(eye(n)-K*H)' + K*R*K';
        P = (P+P')/2;
        dX = dX_ + K * ( dz-H*dX_);   %Updated state mean

        P_VB=P;
        dX_VB=dX;
        v=dz-H*dX_VB;
%         beta1 = H1*(0.5*beta_ + 0.5*0.5*H(:,1:9)'*(v.^2 + diag(H*P_VB*H')))/i;
         beta1 = H1*(beta_ + 0.5*H(:,1:9)'*(v.^2 + diag(H*P_VB*H')));
    end
    beta=beta1+beta2;
    R=abs(diag(beta./alpha_));
    
    
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