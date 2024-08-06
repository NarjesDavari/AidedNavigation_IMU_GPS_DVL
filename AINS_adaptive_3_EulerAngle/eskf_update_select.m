function [Simulation,dX,P,flag_Qadapt,alpha,beta]=eskf_update_select(Simulation,dX,P,dz,H,H2,M,R,I,V,Selection_Param,Include_R_adaptive,Include_Q_adaptive,Include_VB_adaptive, Include_IF_VB_adaptive,Include_HinfVB,gama2,alpha,beta,alphaN,betaN)
        
           windowSize_adaptive= Simulation.Output.Kalman_mtx.WindowSize_adaptive;

           update_counter=Simulation.Output.Kalman_mtx.update_counter;
            Simulation.Output.Kalman_mtx.X_hat_minus(update_counter,:)=dX;%%%?????????
           Simulation.Output.Kalman_mtx.V(Simulation.Output.Kalman_mtx.update_counter,:) = V';

        if Include_VB_adaptive
              [Simulation,dX ,P,alpha,beta] = eskf_update_VB(Simulation,dX , P , dz , H,M,I,alpha,beta);
%               [Simulation,dX ,P,alpha,beta] = eskf_update_VB_2(Simulation,dX , P , dz , H,M,I,alpha,beta);
%             [Simulation]=R_adaptive(Simulation,R,windowSize_adaptive,update_counter,I,Selection_Param);            
              Simulation.Output.Kalman_mtx.X_hat_plus(update_counter,:)=dX;
%             Simulation.Output.Kalman_mtx.P_hat_plus(:,:,update_counter)=P;
              Simulation.Output.Kalman_mtx.A_adap(:,:,update_counter)=Simulation.Output.Kalman_mtx.A;
%               Simulation.Output.Kalman_mtx.alpha_VB=alpha;
%               Simulation.Output.Kalman_mtx.beta_VB=beta;
        elseif Include_HinfVB
               [Simulation,dX ,P,alpha,beta] = eskf_update_HinfVB (Simulation,dX , P , dz , H,M,I,alpha,beta,gama2);
%             [Simulation]=R_adaptive(Simulation,R,windowSize_adaptive,update_counter,I,Selection_Param);            
              Simulation.Output.Kalman_mtx.X_hat_plus(update_counter,:)=dX;
%             Simulation.Output.Kalman_mtx.P_hat_plus(:,:,update_counter)=P;
              Simulation.Output.Kalman_mtx.A_adap(:,:,update_counter)=Simulation.Output.Kalman_mtx.A;
%               Simulation.Output.Kalman_mtx.alpha_VB=alpha;
%               Simulation.Output.Kalman_mtx.beta_VB=beta;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        elseif Simulation.select_adaptive.VB_adaptiveQR 
            [Simulation,dX,P,u_hat,U_hat,P_hat,t_hat]= VB_QRestimat(Simulation,dX,P, dz,H,M, I, update_counter);
            Simulation.Output.Kalman_mtx.VB_adaptiveQR.u_hat(update_counter+1,1)=u_hat;
            Simulation.Output.Kalman_mtx.VB_adaptiveQR.U_hat(:,:,update_counter+1)=U_hat;
            Simulation.Output.Kalman_mtx.VB_adaptiveQR.P_hat(:,:,update_counter)=P_hat;
            Simulation.Output.Kalman_mtx.VB_adaptiveQR.t_hat(update_counter+1,1) = t_hat;
%             Simulation.Output.Kalman_mtx.VB_adaptiveQR.R_hat(:,:,update_counter)=R_hat;
            Simulation.Output.Kalman_mtx.X_hat_plus(update_counter,:)=dX;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        elseif Simulation.select_adaptive.Hinf_VB_adaptiveQR
%             if update_counter==1
%               P_hat = P;
%             else
%             P_hat = Simulation.Output.Kalman_mtx.VB_adaptiveQR.P_hat(:,:,update_counter-1);
%             end
             [Simulation,dX,P,u_hat,U_hat,P_hat,t_hat]= Hinf_VB_QRestimat(Simulation,dX,P, dz,H,M, I, update_counter,gama2);
            Simulation.Output.Kalman_mtx.VB_adaptiveQR.u_hat(update_counter+1,1)=u_hat;
            Simulation.Output.Kalman_mtx.VB_adaptiveQR.U_hat(:,:,update_counter+1)=U_hat;
            Simulation.Output.Kalman_mtx.VB_adaptiveQR.P_hat(:,:,update_counter)=P_hat;
            Simulation.Output.Kalman_mtx.VB_adaptiveQR.t_hat(update_counter+1,1) = t_hat;
%             Simulation.Output.Kalman_mtx.VB_adaptiveQR.R_hat(:,:,update_counter)=R_hat;
            Simulation.Output.Kalman_mtx.X_hat_plus(update_counter,:)=dX;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
        elseif Include_IF_VB_adaptive
              [Simulation,dX,P,alpha,beta] = update_IF_VB(Simulation,dX,dz,H,alpha,beta,alphaN,betaN,I);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
        elseif  Simulation.Output.Kalman_mtx.update_counter >windowSize_adaptive && Include_R_adaptive
            Simulation.Output.Kalman_mtx.adaptive_innovation=1;
           [Simulation,C]= Covariance_innovation(Simulation,windowSize_adaptive);
           [Simulation,dX,P] = eskf_update_adaptive_innovation(Simulation,dX,P,dz,H,I,windowSize_adaptive,C,Selection_Param);
%          [X,P,landa_P] = ekf_update1_adaptive2(Simulation,X,P,Y,H,I,windowSize_adaptive,C);

           Simulation.Output.Kalman_mtx.X_hat_plus(update_counter,:)=dX;
           Simulation.Output.Kalman_mtx.P_hat_plus(:,:,update_counter)=P;
           Simulation.Output.Kalman_mtx.A_adap(:,:,update_counter)=Simulation.Output.Kalman_mtx.A;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                     
        else
%            [dX , P] = eskf_update(dX , P , dz , H, H2, M , R,I );
           
           A=Simulation.Output.Kalman_mtx.A;
           Q=Simulation.Output.Kalman_mtx.Q; 
           [dX,P] = eskf_update_Hinf(A,Q,dX,P,dz,H,H2,M,R,gama2,I);
%            [Simulation]=R_adaptive(Simulation,R,windowSize_adaptive,update_counter,I,Selection_Param,flag_accelrollpitch);

%            Simulation.Output.Kalman_mtx.X_hat_plus(update_counter,:)=dX;
%          Simulation.Output.Kalman_mtx.P_hat_plus(:,:,update_counter)=P;
%            Simulation.Output.Kalman_mtx.A_adap(:,:,update_counter)=Simulation.Output.Kalman_mtx.A;
        end
            
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Q_adaptive
           
           if  Simulation.Output.Kalman_mtx.update_counter >windowSize_adaptive && Include_Q_adaptive
               [Simulation] = Estimate_Q(Simulation);
               flag_Qadapt=1;
           else
               flag_Qadapt=0;
           end
               
           
           
           
           