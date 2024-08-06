%Integrated Navigation System using EKF and UKF
function [Simulation]=IINS_1(Simulation,coeff_final,include_MA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fs                       = Simulation.Init_Value.fs;
include_GPS              = Simulation.Auxiliary_Snsr.include_GPS;
include_depthmeter       = Simulation.Auxiliary_Snsr.include_depthmeter;
include_dvl              = Simulation.Auxiliary_Snsr.include_dvl;
include_heading          = Simulation.Auxiliary_Snsr.include_heading;
include_accelrollpitch   = Simulation.Auxiliary_Snsr.include_accelrollpitch;

Include_R_adaptive= Simulation.select_adaptive.MAKF.R_adaptive;
Include_Q_adaptive= Simulation.select_adaptive.MAKF.Q_adaptive;
Include_VB_adaptive=Simulation.select_adaptive.VB;
Include_IF_VB_adaptive= Simulation.select_adaptive.IF_VB_adaptive;

fc_f_RP                  = Simulation.Parameters_Inclenometer.fc_f_RP;
fc_f_accel               = Simulation.Parameters_Accel.fc_f_accel;
fc_f_gyro                = Simulation.Parameters_Gyro.fc_f_gyro;
muu                      = Simulation.Parameters_Inclenometer.mu;

filtered_accel           = Simulation.Parameters_Accel.filtered_accel;
filtered_gyro            = Simulation.Parameters_Gyro.filtered_gyro;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global frp;
global Accel_Count;
global Attitude_Valid;
global Roll_Sum;
global Pitch_Sum;
frp           = 10;
Accel_Count   = 0;
Attitude_Valid= 0;
Roll_Sum      = 0;
Pitch_Sum     = 0;
dt=1/fs;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global Cbn_det;
global calib_num ;
global GPS_calib_count ;
global depth_calib_count ;
global DVL_calib_count ;
global Hdng_calib_count ;
calib_num = 100 / Simulation.Init_Value.calib_freq;
GPS_calib_count = 0;
depth_calib_count = 0;
DVL_calib_count = 0;
Hdng_calib_count = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SF = Simulation.Init_Value.DVL_SF;
Misalignment_phins_DVL=Simulation.Parameters_Misalignment.phins_DVL;
Misalignment_IMU_phins=Simulation.Parameters_Misalignment.IMU_phins;
C_imu_phins=InCBN(Simulation.Parameters_Misalignment.IMU_phins*pi/180);
C_phins_DVL=InCBN(Simulation.Parameters_Misalignment.phins_DVL*pi/180);
C_DVL_IMU=eye(3); %%%C_imu_phins*C_phins_DVL;

ave_sample = Simulation.Init_Value.ave_sample;
calib_sample = Simulation.Init_Value.calib_sample;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% calculate the time of data loos
Dlength = length(Simulation.Input.Measurements.IMU);
include_DataLose_GPS =Simulation.Parameters_GPS.Include_Data_Lose;
        if include_DataLose_GPS 
            Number_dataLose= Simulation.Parameters_GPS.Number_Data_Los;
            time_outage  = Simulation.Parameters_GPS.interval_outage;
            Start_Time_out  = Simulation.Parameters_GPS.Time_start_outage;
            [Simulation,t1,t2]=Data_Lose_GPS(Simulation,Number_dataLose,time_outage,Start_Time_out,Dlength,ave_sample);
             Simulation.Output.ESKF.Pos_Error.GPS_Outage.t1=t1;
             Simulation.Output.ESKF.Pos_Error.GPS_Outage.t2=t2;
             select_func.Navigate_Error_Datalose_GPS=1;
        end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DVL
include_DataLose_DVL =Simulation.Parameters_DVL.Include_Data_Lose;
        if include_DataLose_DVL 
            Number_dataLose= Simulation.Parameters_DVL.Number_Data_Los;
            time_outage  = Simulation.Parameters_DVL.interval_outage;
            Start_Time_out  = Simulation.Parameters_DVL.Time_start_outage;
            [Simulation,t1,t2]=Data_Lose_DVL(Simulation,Number_dataLose,time_outage,Start_Time_out,Dlength,ave_sample);
             Simulation.Output.ESKF.Pos_Error.DVL_Outage.t1=t1;
             Simulation.Output.ESKF.Pos_Error.DVL_Outage.t2=t2;
             select_func.Navigate_Error_Datalose_DVL=1;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%parameter of guass markov

%  taux = GetParam(Simulation.Parameters_Accel ,'accel_correlation_time');
%  tauy = GetParam(Simulation.Parameters_Gyro ,'gyro_correlation_time');             
% tau=[taux tauy];%tau=[tau_ax,tau_ay,tau_az,tau_gx,tau_gy,tau_gz];
% Simulation.Output.parameter_Bias_tauStation=tau;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=1/fs;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        select_func.conversion      = 1;
        select_func.distance_cacul  = 1;
        select_func.Navigate_Error  = 1;  
        select_func.Navigate_Error_Datalose=1;

      
       %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
       %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                 func_f  = @Navigation_f;
                %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                    [ Simulation ] = Qc_setting( Simulation );
                    [ Simulation ] = R_setting( Simulation );
                %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                tic
                    Dlength=length(Simulation.Input.Measurements.IMU);      
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    Number_adaptive=20;
                    Simulation.Output.Kalman_mtx.WindowSize_adaptive= Number_adaptive;
                    
                    [ Simulation,flag_Qadapt ] = Initialization( Simulation,fs,Include_Q_adaptive,ave_sample);
                    landa_P= Simulation.Output.Kalman_mtx.landa_P;
                    Simulation.Output.Kalman_mtx.Number_adaptive= Number_adaptive;
                    Selection_Param1 ={include_GPS,include_depthmeter,include_dvl,include_heading,include_accelrollpitch ,muu };
                  
                   I=ave_sample+1;
                   while I <= Dlength
                       dt = Simulation.Input.Measurements.IMU(I,1)-Simulation.Input.Measurements.IMU(I-1,1);
                       I1=I;
                        while dt<=0
                            I=I+1;
                            dt = Simulation.Input.Measurements.IMU(I,1)-Simulation.Input.Measurements.IMU(I1,1);
%                            dt=0.05;   
                        end
                        I=I;
                            fs=1/dt;
                        IMU_Time   = num2str(Simulation.Input.Measurements.IMU(I,1));
%                         IMU_Time   = (floor(Simulation.Input.Measurements.IMU(I,1)*100))/100;
                         if Simulation.Input.Measurements.GPS_Counter < length(Simulation.Input.Measurements.GPS)
                            GPS_Time   = num2str(Simulation.Input.Measurements.GPS(Simulation.Input.Measurements.GPS_Counter,1));
%                             GPS_Time   = (floor(Simulation.Input.Measurements.GPS(Simulation.Input.Measurements.GPS_Counter,1)*100))/100;
                        end
                        if Simulation.Input.Measurements.Depth_Counter < length(Simulation.Input.Measurements.Depth)
                            depth_Time = num2str(Simulation.Input.Measurements.Depth(Simulation.Input.Measurements.Depth_Counter,1));
%                             depth_Time = (floor(Simulation.Input.Measurements.Depth(Simulation.Input.Measurements.Depth_Counter,1)*100))/100;
                        end
                        if Simulation.Input.Measurements.DVL_Counter < length(Simulation.Input.Measurements.DVL)
                            DVL_Time   = num2str(Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,1));
%                             DVL_Time   = (floor(Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,1)*100))/100;
                        end
%                         if Simulation.Input.Measurements.incln_Counter < length(Simulation.Input.Measurements.RollPitch)
%                             incln_Time = num2str(Simulation.Input.Measurements.RollPitch(Simulation.Input.Measurements.incln_Counter,1,1));
%                         end
                        if Simulation.Input.Measurements.hdng_Counter < length(Simulation.Input.Measurements.Heading)
                            hdng_Time  = num2str(Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,1));
%                             hdng_Time  = (floor(Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,1)*100))/100;
                        end 

                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
                        
                        
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
                        Sensors_Time = {IMU_Time , GPS_Time , depth_Time , DVL_Time ,hdng_Time}; 
                        Selection_Param =[Sensors_Time,Selection_Param1 ];
                        [Simulation]=SINS_2(Simulation , I , dt ,func_f, fc_f_RP , fc_f_accel , fc_f_gyro, filtered_accel , filtered_gyro ,ave_sample,Selection_Param{11},coeff_final,include_MA);                 
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                               
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        [Simulation,flag_Qadapt] = ESKF(Simulation,I,Selection_Param,C_DVL_IMU,landa_P,Include_R_adaptive,Include_Q_adaptive,Include_VB_adaptive, Include_IF_VB_adaptive,flag_Qadapt,Misalignment_IMU_phins,ave_sample,calib_sample,SF,dt);
                        [Simulation] = State_Correction( Simulation , I, ave_sample )  ;                      
                        x = [Simulation.Output.INS.X_INS(I-ave_sample,1:3),Simulation.Output.INS.X_INS(I-ave_sample,4:6),Simulation.Output.INS.fnn(I-ave_sample,:)]; 
                        C = Simulation.Output.INS.Cbn(:,:,I-ave_sample);
                        [Simulation,flag_Qadapt]=AQ_calcul(x,C,Simulation,fs,I,Include_Q_adaptive,flag_Qadapt);  
                    
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        
                         if I== calib_sample
% %                             [ Simulation ] = R_Moving( Simulation );
                             [  Simulation , include_GPS , include_depthmeter , include_dvl , include_heading , include_accelrollpitch ] = Moving_Selection(Simulation);
%                           
                             Selection_Param1 = {include_GPS , include_depthmeter , include_dvl , include_heading , include_accelrollpitch , muu };
%                    
                         end
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        if rem(I-ave_sample+1,10000)==0
                            I - ave_sample + 1
                        end                        
                    end
                     Simulation.Output.ESKF.X_i(:,:)=Simulation.Output.INS.X_INS(:,1:3);
                                 
                %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                a=toc;
                a_min=fix(a)/60;
                a_sec=(a_min-fix(a_min))*60;
                fprintf('Elapsed time : %f min ',fix(a_min));
                fprintf('%f sec\n ',a_sec);                  
                %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                Simulation.Output.ESKF.Cbn_det = Cbn_det;
                %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                if select_func.conversion
                    [Simulation]=conversion(Simulation , ave_sample);
                end         
                %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                if select_func.distance_cacul
                    [ Simulation ] = distance_cacul( Simulation, ave_sample );
                end
                %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                if select_func.Navigate_Error
                    [Simulation]=Navigate_Error(Simulation ,  ave_sample);
                end
                if select_func.Navigate_Error_Datalose && include_DataLose_GPS 
                   [ Simulation] = Navigate_Error_Datalose_GPS ( Simulation, ave_sample );
                end
                %%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                if  select_func.Navigate_Error_Datalose && include_DataLose_DVL 
                   [ Simulation] = Navigate_Error_Datalose_DVL ( Simulation,ave_sample );
                end
                %%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
               fprintf('Travelled time : %.2f min\n',Simulation.Output.ESKF.Pos_Error.travelled_time/60);
                fprintf('Travelled distance : %.2f meter\n',Simulation.Output.ESKF.Pos_Error.travelled_distance);
                fprintf('Travelled distance in x direction : %.2f meter\n',Simulation.Output.ESKF.Pos_Error.travelled_distancex);
                fprintf('Travelled distance in y direction : %.2f meter\n',Simulation.Output.ESKF.Pos_Error.travelled_distancey);
                fprintf('Travelled distance in z direction : %.2f meter\n',Simulation.Output.ESKF.Pos_Error.travelled_distancez);
                
                fprintf('RMSE : %.3f meter\n',Simulation.Output.ESKF.Pos_Error.RMSE);
                fprintf('Relative RMSE : %.3f %%\n',Simulation.Output.ESKF.Pos_Error.Relative_RMSE);
                
                fprintf('RMSE in x direction : %.3f meter\n',Simulation.Output.ESKF.Pos_Error.RMSEx);
                fprintf('RMSE in y direction : %.3f meter\n',Simulation.Output.ESKF.Pos_Error.RMSEy);
                fprintf('RMSE in z direction : %.3f meter\n',Simulation.Output.ESKF.Pos_Error.RMSEz);

                fprintf('Relative RMSE in x direction : %.3f %%\n',Simulation.Output.ESKF.Pos_Error.Relative_RMSEx);
                fprintf('Relative RMSE in y direction : %.3f %%\n',Simulation.Output.ESKF.Pos_Error.Relative_RMSEy);
                fprintf('Relative RMSE in z direction : %.3f %%\n',Simulation.Output.ESKF.Pos_Error.Relative_RMSEz);
                
                fprintf('Absolute error in end of path : %.3f meter\n',Simulation.Output.ESKF.Pos_Error.absolute_error_end);
                fprintf('Relative error in end of path : %.3f %%\n',Simulation.Output.ESKF.Pos_Error.relative_error_end);
end
                
       


