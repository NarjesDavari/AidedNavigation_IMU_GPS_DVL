%Error State Kalman Filter function:
%prediction and correction stages of the Kalman filter
function [Simulation,flag_Qadapt] = ESKF( Simulation,I,Selection_Param,C_DVL_IMU,Include_R_adaptive,Include_Q_adaptive,Include_VB_adaptive,Include_IF_VB_adaptive,Include_HinfVB,gama2,flag_Qadapt,Misalignment_IMU_phins,ave_sample,calib_sample,SF,dt,fs_auxiliarySensor)

global Updt_Cntr;
    
if Include_IF_VB_adaptive
    [Simulation,dX,P,alpha,beta] = predict_IF_VB (Simulation,I);
elseif Include_HinfVB %%&& Updt_Cntr >=1
    [Simulation,dX,P,alpha,beta] = predict_HinfVB (Simulation,gama2,I,ave_sample);
else
    [Simulation,dX,P,alpha,beta] = eskf_predict(Simulation,I,ave_sample,Updt_Cntr);
end

if Simulation.parameter_outlierDetDVL.active==1
    [Simulation,V,alphaN,betaN ] = Correction_Param_outlierDVL(Simulation,Selection_Param,SF,I,C_DVL_IMU,P,Misalignment_IMU_phins,ave_sample,calib_sample,alpha,beta,dt);
else
    [Simulation,V,alphaN,betaN ]=Correction_Param_2(Simulation,Selection_Param,SF,I,C_DVL_IMU,P,Misalignment_IMU_phins,ave_sample,calib_sample,alpha,beta,dt,fs_auxiliarySensor);
end
        H = Simulation.Output.Kalman_mtx.H;
        H2 = Simulation.Output.Kalman_mtx.H2;
        M = Simulation.Output.Kalman_mtx.M;
        R = Simulation.Output.Kalman_mtx.R.Rmatrx;
        dz = Simulation.Output.Kalman_mtx.dz;

        if (~isempty (Simulation.Output.Kalman_mtx.H))
            Updt_Cntr = Updt_Cntr + 1;
           [Simulation,dX,P,flag_Qadapt,alpha,beta]=eskf_update_select(Simulation,dX,P,dz,H,H2,M,R,I,V,Selection_Param,Include_R_adaptive,Include_Q_adaptive,Include_VB_adaptive, Include_IF_VB_adaptive,Include_HinfVB,gama2,alpha,beta,alphaN,betaN);          
           Simulation.Output.Kalman_mtx.alpha_VB=alpha;
           Simulation.Output.Kalman_mtx.beta_VB=beta;
           Simulation.Output.Kalman_mtx.update_counter = Simulation.Output.Kalman_mtx.update_counter+1;
        end
        
        Simulation.Output.Kalman_mtx.alpha_VB=alpha;
        Simulation.Output.Kalman_mtx.beta_VB=beta;
        Simulation.Output.Kalman_mtx.P = P;
        Simulation.Output.ESKF.dX(I - ave_sample + 1,:) = dX;
        Q_update=Simulation.Output.Kalman_mtx.Q;
%         Simulation.Output.Kalman_mtx.Q_diag(I - ave_sample + 1,:)=[Q_update(1,1) Q_update(2,2) Q_update(3,3) Q_update(4,4) Q_update(5,5) Q_update(6,6) Q_update(7,7) Q_update(8,8) Q_update(9,9) Q_update(10,10) Q_update(11,11) Q_update(12,12) Q_update(13,13) Q_update(14,14) Q_update(15,15)];

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         P1=zeros(15,15);
%         P1(1:9,1:9)=P(1:9,1:9);
%         Simulation.Output.Kalman_mtx.P=P1;
    