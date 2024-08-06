%Computation of The measurement model matrix(H) and the measurement error
%variance(R) in direct approch (EKF).
function [ Simulation ] = EKF_Correction_Param(Simulation,Selection_Param ,I,C_DVL_IMU,ave_sample,dt )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
IMU_Time = Selection_Param{1};
GPS_Time = Selection_Param{2};
depth_Time = Selection_Param{3};
DVL_Time   = Selection_Param{4};
% incln_Time = Selection_Param{5};
hdng_Time  = Selection_Param{5};
include_GPS = Selection_Param{6};
include_depthmeter = Selection_Param{7};
include_dvl = Selection_Param{8};
include_heading = Selection_Param{9};
include_rollpitch = 0;
include_accelrollpitch = Selection_Param{10};
mu = Selection_Param{11};
H=[];
Rr=[];
z=[];
M=[];
H2=[];
H1=[];
b=0;
coef_dt = 1;
delta_t=0.011;
SF =Simulation.Init_Value.DVL_SF;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global GPS_fusion_active ;
global depth_fusion_active ;
global DVL_fusion_active ;
global accelrollpitch;
global hdng_fusion_active;
global rollpitch_fusion_active;
GPS_fusion_active   = 0;
depth_fusion_active = 0;
DVL_fusion_active   = 0;
rollpitch_fusion_active =0;
accelrollpitch =0;
hdng_fusion_active=0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if include_GPS
        
        while str2double(GPS_Time)< str2double(IMU_Time)
                Simulation.Input.Measurements.GPS_Counter  = Simulation.Input.Measurements.GPS_Counter + 1;
               GPS_Time  = num2str((round(Simulation.Input.Measurements.GPS(Simulation.Input.Measurements.GPS_Counter,1)*1000))/1000);
        end
            if abs(str2double(IMU_Time)-str2double(GPS_Time))<=delta_t
                H=[H;
                       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0 
                       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0 ];
  
                Rr=[Rr;
                       Simulation.Output.Kalman_mtx.R.r_Lat;
                       Simulation.Output.Kalman_mtx.R.r_lon];
                
                GPS = Simulation.Input.Measurements.GPS(Simulation.Input.Measurements.GPS_Counter,2:3)*(pi/180);

                z = [z,GPS];
                GPS_fusion_active=1;
                H1=[H1,1 1];
                H2=[H2,0 0];
            else
            H1=[H1,0 0];
            H2=[H2,1 1];
            end
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if abs(str2double(IMU_Time)-str2double(GPS_Time))<=delta_t%%*(1/fs_GPS) %%%strcmp(IMU_Time,GPS_Time) %%str2double(GPS_Time)<=str2double(IMU_Time)  %%GPS_Time <= IMU_Time  %%%  ||  %%%   %% 
            Simulation.Input.Measurements.GPS_Counter   = Simulation.Input.Measurements.GPS_Counter + 1;
        end
        if GPS_fusion_active
            Simulation.Input.Measurements.GPS_Counter_fusion   = Simulation.Input.Measurements.GPS_Counter_fusion + 1;
        end
    else
        H1=[H1,0 0];
        H2=[H2,1 1];
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   if include_depthmeter
       
       while str2double(depth_Time)< str2double(IMU_Time)
                Simulation.Input.Measurements.Depth_Counter  = Simulation.Input.Measurements.Depth_Counter + 1;
               depth_Time  = num2str((round(Simulation.Input.Measurements.Depth(Simulation.Input.Measurements.Depth_Counter,1)*1000))/1000);
       end
           if abs(str2double(IMU_Time)-str2double(depth_Time))<=delta_t
                H = [H;
                       0,0,1,0,0,0,0,0,0,0,0,0,0,0,0];
                Rr = [Rr;
                         Simulation.Output.Kalman_mtx.R.r_alt];
                depth = Simulation.Input.Measurements.Depth(Simulation.Input.Measurements.Depth_Counter,2);
                
                z = [z,depth];
                 depth_fusion_active=1;
                H1=[H1,1];
                H2=[H2,0];
           else
            H1=[H1,0];
            H2=[H2,1];
            end
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         if abs(str2double(IMU_Time)-str2double(depth_Time))<=delta_t%%*(1/fs_Depth) %%%strcmp(IMU_Time,depth_Time)%%str2double(depth_Time)<=str2double(IMU_Time) %%%depth_Time <= IMU_Time %% ||  %%
                    Simulation.Input.Measurements.Depth_Counter = Simulation.Input.Measurements.Depth_Counter + 1;
         end 
         if depth_fusion_active
            Simulation.Input.Measurements.Depth_Counter_fusion   = Simulation.Input.Measurements.Depth_Counter_fusion + 1;
         end 
   else
       H1=[H1,0];
       H2=[H2,1];
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if include_dvl
        if abs(str2double(IMU_Time)-str2double(DVL_Time))> coef_dt*dt
                while str2double(DVL_Time)< str2double(IMU_Time)
                        Simulation.Input.Measurements.DVL_Counter  = Simulation.Input.Measurements.DVL_Counter + 1;
                       DVL_Time  = num2str((round(Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,1)*1000))/1000);
                end
        end
             if abs(str2double(IMU_Time)-str2double(DVL_Time))<=delta_t
                H=[H;
                    zeros(3),eye(3),zeros(3,9)];
                
                Rr=[Rr;
                    Simulation.Output.Kalman_mtx.R.r_vx;
                    Simulation.Output.Kalman_mtx.R.r_vy;
                    Simulation.Output.Kalman_mtx.R.r_vz];   
                
                 vb = (1+(SF)/100)*C_DVL_IMU * Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,2:4)';    
                      
                 vn = -(Simulation.Output.EKF.Cbn(:,:,I)*vb)';
                 z  = [z , vn];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
              DVL_fusion_active=1;
              H1=[H1,1 1 1];
              H2=[H2,0 0 0];
             else
                 H1=[H1,0 0 0];
                 H2=[H2,1 1 1];
            end
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         if  abs(str2double(IMU_Time)-str2double(DVL_Time))<=delta_t%%*(1/fs_DVL) %%%strcmp(IMU_Time,DVL_Time) %%%DVL_Time <= IMU_Time %% || %% strcmp(IMU_Time,DVL_Time) %%
                Simulation.Input.Measurements.DVL_Counter       = Simulation.Input.Measurements.DVL_Counter + 1;
        end
        if DVL_fusion_active
            Simulation.Input.Measurements.DVL_Counter_fusion   = Simulation.Input.Measurements.DVL_Counter_fusion + 1;
        end
    else
        H1=[H1,0 0 0];
        H2=[H2,1 1 1];
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if include_rollpitch
            if strcmp(IMU_Time,incln_Time)
                rollpitch_fusion_active =1;
                i_phi=[1 0 0
                       0 1 0];
                H=[H;
                    zeros(2,6),i_phi];
                Rr=[Rr;
                    Simulation.Output.Kalman_mtx.R.r_roll;
                    Simulation.Output.Kalman_mtx.R.r_pitch];   
                   
                z = [z , Simulation.Input.Measurements.RollPitch(Simulation.Input.Measurements.incln_Counter,2,i)*pi/180,...
                         Simulation.Input.Measurements.RollPitch(Simulation.Input.Measurements.incln_Counter,3,i)*pi/180];
                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                   Simulation.Input.Measurements.incln_Counter = Simulation.Input.Measurements.incln_Counter + 1;
            end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if include_accelrollpitch
            if Simulation.Output.Alignment.norm.Accel_norm(I) <  mu 
                i_phi=[1 0 0
                       0 1 0];
                H=[H;
                    zeros(2,6),i_phi];
                Rr=[Rr;
                    Simulation.Output.Kalman_mtx.R.r_aroll;
                    Simulation.Output.Kalman_mtx.R.r_apitch];   
                   
                z=[z, Simulation.Output.Alignment.phi(Simulation.Output.Alignment.RP_counter,1),...
                      Simulation.Output.Alignment.theta(Simulation.Output.Alignment.RP_counter,1)];
                  
                  accelrollpitch=1;
                  H1=[H1,1 1];
                 H2=[H2,0 0];
                 b=1;
            end
    end    
     if b~=1  
       H1=[H1,0 0];
       H2=[H2,1 1];
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if include_heading
         while str2double(hdng_Time)< str2double(IMU_Time)
                Simulation.Input.Measurements.hdng_Counter  = Simulation.Input.Measurements.hdng_Counter + 1;
               hdng_Time  = num2str((round(Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,1)*1000))/1000);
         end
            if   abs(str2double(IMU_Time)-str2double(hdng_Time))<=delta_t
                i_psi=[0 0 1];     
                C = Gamma(Simulation.Output.EKF.X(I - ave_sample + 1,7:9));
                H=[H;
                    zeros(1,6),i_psi/C,zeros(1,6)];

                Rr=[Rr;
                    Simulation.Output.Kalman_mtx.R.r_yaw];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,2)*pi/180 > pi
                    z_hdng = (Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,2)*pi/180) - 2*pi;
                elseif Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,2)*pi/180 < -pi
                    z_hdng = (Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,2)*pi/180) + 2*pi;
                else
                    z_hdng = Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,2)*pi/180;
                end                        
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                    
                z = [z , z_hdng]; 
               hdng_fusion_active=1;
               H1=[H1,1];
                H2=[H2,0];
            end
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if  abs(str2double(IMU_Time)-str2double(hdng_Time))<=delta_t%%*(1/fs_heading) %%% str2double(hdng_Time)<=str2double(IMU_Time) %%%hdng_Time <= IMU_Time %%% ||  %%strcmp(IMU_Time,hdng_Time) %%%
                Simulation.Input.Measurements.hdng_Counter  = Simulation.Input.Measurements.hdng_Counter + 1;
        end
         if hdng_fusion_active 
            Simulation.Input.Measurements.hdng_Counter_fusion   = Simulation.Input.Measurements.hdng_Counter_fusion + 1;
         end 
    else
      H1=[H1,0];
      H2=[H2,1];
    end 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     M= [eye(3),zeros(3,6);zeros(3),Simulation.Output.EKF.Cbn(:,:,I - ave_sample + 1),zeros(3);zeros(3,6),eye(3)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    R=diag(Rr);%measurement noise covariance matrix
    Simulation.Output.Kalman_mtx.H=0;
    Simulation.Output.Kalman_mtx.H1=0;
    Simulation.Output.Kalman_mtx.H2=0;
    Simulation.Output.Kalman_mtx.M=0;
    Simulation.Output.Kalman_mtx.R.Rmatrx=0;
    Simulation.Output.Kalman_mtx.z=0;
    Simulation.Output.Kalman_mtx.H=H;%measurement matrix
    Simulation.Output.Kalman_mtx.H1=diag(H1);
    Simulation.Output.Kalman_mtx.H2=diag(H2);
    Simulation.Output.Kalman_mtx.M=M;
    Simulation.Output.Kalman_mtx.R.Rmatrx=R; 
    Simulation.Output.Kalman_mtx.z=z';   
end
%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
