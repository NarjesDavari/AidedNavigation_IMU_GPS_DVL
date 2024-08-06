%IF_UPDATE  information Filter update step

function [Simulation,dX,P,alphaN,betaN] = update_IF_VB(Simulation,dX_,dz,H,alpha_,beta_,alpha,beta,I)
% function [Simulation,dX,P,alpha,beta] = eskf_update_VB(Simulation,dX_,P_,dz,H,H2,M,I,alpha_,beta_,flag_accelrollpitch,windowsize)
update_counter=Simulation.Output.Kalman_mtx.update_counter;

    H1=Simulation.Output.Kalman_mtx.H1;
    H2=Simulation.Output.Kalman_mtx.H2;
    Y= Simulation.Output.Kalman_mtx.Y; %%% information matrix
    y = Simulation.Output.Kalman_mtx.y;
    
% % global GPS_fusion_active ;
% % global depth_fusion_active ;
% % global DVL_fusion_active ;
% % global Hdng_fusion_active ;

global Updt_Cntr;
N=3;
% c=(A*P_*A'+G*Qc*G');
% alpha = alpha_; %*eye(length(alpha0));
% beta = beta_;

% for j=1:length(diag(H1))
   r=find (diag(H1)==0,9); 
% end
alpha=alpha+1/2;
beta1=beta;
% beta2=H2*beta_;
betaN=[];
alphaN=[];
    for i=1:N
        if Updt_Cntr==1
            R=Simulation.Output.Kalman_mtx.R.Rmatrx;
%             S=H(:,1:9)*(M')*H(:,1:9)'*R*H(:,1:9)*(M)*H(:,1:9)' + H*P_*H';
        else
            R=diag(beta./alpha);
%             S=H(:,1:9)*M*R*M'*H(:,1:9)' + H*P_*H';
        end 
        y = y + H' * R^(-1) * dz; 
        Y = Y + H' * R^(-1) * H; %%% S
        [u,d,v1]=svd(Y);
        Y = u*sqrt(d)*u';  
        
        P = Y^(-1);
        dX = P * y;
        v=dz-H*dX_;
        beta = (beta1+ 0.5*(v.^2 + diag(H*P*H')))/100;
    end
    Simulation.Output.Kalman_mtx.Y = Y; %%% information matrix
    Simulation.Output.Kalman_mtx.y = y; 
    Simulation.Output.Kalman_mtx.X_hat_plus(update_counter,:)=dX;
    Simulation.Output.Kalman_mtx.P_hat_plus(update_counter,:)=diag(P);
    
    k=1;
    l=1;
    flag=0;
%     if (~isempty (r))
%         for j=1:length(alpha_)
%             %         if  (~isempty (r))
%             if j==r(k)
%                 alphaN(j,1)=alpha_(r(k));
%                 betaN(j,1)=beta_(r(k));
%                 flag=1;
%                 if k<length(r)
%                     k=k+1;
%                 else
%                     flag=0;
%                 end
%                 
%             end
%             %         end
%             if j~=r(k) && flag==0
%                 alphaN(j,1)=alpha(l);
%                 betaN(j,1)=beta(l);
%                 if l<length(alpha)
%                     l=l+1;
%                 end
%             end
%         end
%     else
%         alphaN=alpha;
%         betaN=beta;
%     end
   H_N=Simulation.Output.Kalman_mtx.H_N;
   alphaN= (alpha'*H_N)';
   betaN= (beta'*H_N)';
    r=find(alphaN==0,9);
    j=1;
    while j<=length(r)
        alphaN(r(j))=alpha_(r(j));
        betaN(r(j))=beta_(r(j));
        j=j+1;
    end
%     Simulation.Output.Kalman_mtx.alpha_VB=alphaN;
%     Simulation.Output.Kalman_mtx.beta_VB=betaN;

%       R=abs(diag(beta./alpha_));

%         K = P_*H'/diag(diag(S));                 %Computed Kalman gain
%         P = P_ - K*H*P_;              %Updated state covariance
%         n = size(P,1);
%         %       P = (eye(n)-K*H)*P*(eye(n)-K*H)' + K*R*K';
%         P = (P+P')/2;
%         dX = dX_ + K * ( dz-H*dX_);   %Updated state mean
% 
%         P_VB=P;
%         dX_VB=dX;
      
%         beta1 = H1*(0.5*beta_ + 0.5*0.5*H(:,1:9)'*(v.^2 + diag(H*P_VB*H')))/i;
    end
%     beta=beta1+beta2;
  
    
    
%     if Updt_Cntr~=1
%         if GPS_fusion_active
%              Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter-1,1)=R(1,1);
%              Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter-1,2)=R(2,2);
%         end
%         if depth_fusion_active
%              Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter-1,1)=R(3,3);
%         end
%         if DVL_fusion_active
%              Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter-1,1)=R(4,4);
%              Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter-1,2)=R(5,5);
%              Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter-1,3)=R(6,6);
%         end
%         if flag_accelrollpitch==1
%              Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter,1)=R(7,7);
%              Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter,2)=R(8,8);
%         end
%         if Hdng_fusion_active
%              Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter-1,1)=R(9,9);
%         end
%     end
   
