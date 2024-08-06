%Computation of The measurement model matrix(H) and the measurement error
%variance(R) in indirect approch (ESKF).
%Reference : My thesis: pages 90-91
function [ Simulation ] = Correction_Param2(Simulation , Selection_Param , Sensors_Time , I , C_IMU_DVL,SF , P , ave_sample ,calib_sample )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global calib_num ;
global GPS_calib_count ;
global depth_calib_count ;
global DVL_calib_count ;
global Hdng_calib_count ;
GPS_calib_count   = GPS_calib_count + 1 ;
depth_calib_count = depth_calib_count + 1 ;
DVL_calib_count   = DVL_calib_count + 1 ;
Hdng_calib_count  = Hdng_calib_count + 1 ;

GPS_fusion_active   = 0;
depth_fusion_active = 0;
DVL_fusion_active   = 0;
Hdng_fusion_active  = 0;
flag=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_Lat = Simulation.Rej_Cof.Lat;
T_lon = Simulation.Rej_Cof.Lon;
Tz    = Simulation.Rej_Cof.Z;

TN = Simulation.Rej_Cof.VN;
TE = Simulation.Rej_Cof.VE;
TD = Simulation.Rej_Cof.VD;
Th = Simulation.Rej_Cof.H;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
IMU_Time               = Sensors_Time{1};
GPS_Time               = Sensors_Time{2};
depth_Time             = Sensors_Time{3};
DVL_Time               = Sensors_Time{4};
hdng_Time              = Sensors_Time{5};

include_GPS            = Selection_Param{1};
include_depthmeter     = Selection_Param{2};
include_dvl            = Selection_Param{3};
include_heading        = Selection_Param{4};
include_accelrollpitch = Selection_Param{5};
mu                     = Selection_Param{6};

H                      = [];
Rr                     = [];
dz                     = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if include_GPS
        if I <= calib_sample && GPS_calib_count >= calib_num
            GPS_fusion_active = 1; 
            GPS_calib_count = 0;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
            d_GPS = Simulation.Output.INS.X_INS(I - ave_sample + 1,1:2) - ...
                    Simulation.Output.INS.X_INS(1,1:2);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        elseif I > calib_sample && strcmp(IMU_Time,GPS_Time) 
            GPS_fusion_active = 1;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
            d_GPS = Simulation.Output.INS.X_INS(I - ave_sample + 1,1:2) - ...
                    Simulation.Input.Measurements.GPS(Simulation.Input.Measurements.GPS_Counter,2:3)*(pi/180);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        if GPS_fusion_active            
            Simulation.Output.Kalman_mtx.dz_gps (Simulation.Input.Measurements.GPS_Counter,:) = d_GPS;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Hgps = [ 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0 
                     0,1,0,0,0,0,0,0,0,0,0,0,0,0,0 ];                
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Rgps = [ Simulation.Output.Kalman_mtx.R.r_Lat;
                     Simulation.Output.Kalman_mtx.R.r_lon];                
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            S = diag(Rgps) + Hgps*P*Hgps';
            Simulation.Output.Kalman_mtx.S_gps(Simulation.Input.Measurements.GPS_Counter,:) = diag(S)';
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Kgps = P*Hgps'/S;
            Simulation.Output.Kalman_mtx.K_gps(Simulation.Input.Measurements.GPS_Counter,:) = [Kgps(1,1),Kgps(2,2)];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if Simulation.Input.Measurements.GPS_Miss_Counter2 < 3
                if (d_GPS(1)^2)>T_Lat*S(1,1) || (d_GPS(2)^2)>T_lon*S(2,2)
                    Simulation.Input.Measurements.GPS_Miss_Counter = Simulation.Input.Measurements.GPS_Miss_Counter + 1;
                    Simulation.Input.Measurements.GPS_Miss_Counter2 = Simulation.Input.Measurements.GPS_Miss_Counter2 + 1;
                else
                    H    = [ H ; Hgps ];
                    Rr   = [Rr; Rgps];
                    dz   = [dz,d_GPS];
                    Simulation.Input.Measurements.GPS_Miss_Counter2 = 0;
                 end
            else
                H    = [ H ; Hgps ];
                Rr   = [Rr; Rgps];
                dz   = [dz,d_GPS];
                Simulation.Input.Measurements.GPS_Miss_Counter2 = 0;              
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if strcmp(IMU_Time,GPS_Time)
            Simulation.Input.Measurements.GPS_Counter   = Simulation.Input.Measurements.GPS_Counter + 1;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    end
    %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    if include_depthmeter
        if I <= calib_sample && depth_calib_count >= calib_num
            depth_fusion_active = 1; 
            depth_calib_count = 0;            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            d_depth = Simulation.Output.INS.X_INS(I - ave_sample + 1,3)- ...
                      Simulation.Output.INS.X_INS(1,3);       
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        elseif I > calib_sample && strcmp(IMU_Time,depth_Time) %if strcmp(IMU_Time,depth_Time)
            depth_fusion_active = 1; 
%             d_depth = Simulation.Output.INS.X_INS(I - ave_sample + 1,3)- ...
%                       Simulation.Input.Measurements.Depth(Simulation.Input.Measurements.Depth_Counter,2); 
            d_depth=Simulation.Output.INS.X_INS(I - ave_sample + 1,3)- 0;            
        end
        if depth_fusion_active
                Simulation.Output.Kalman_mtx.dz_depth (Simulation.Input.Measurements.Depth_Counter,1) = d_depth;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                Hdepth = [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0];                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                Rdepth = Simulation.Output.Kalman_mtx.R.r_alt;                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
                S = Rdepth + Hdepth*P*Hdepth';
                Simulation.Output.Kalman_mtx.S_depth(Simulation.Input.Measurements.Depth_Counter,:) = diag(S)';                  
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                Kdepth = P*Hdepth'/S;
                Simulation.Output.Kalman_mtx.K_depth(Simulation.Input.Measurements.Depth_Counter,:) = Kdepth(3,1);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if Simulation.Input.Measurements.Depth_Miss_Counter2 < 3
                     if (d_depth^2)>Tz*S
                        Simulation.Input.Measurements.Depth_Miss_Counter  = Simulation.Input.Measurements.Depth_Miss_Counter + 1;
                        Simulation.Input.Measurements.Depth_Miss_Counter2 = Simulation.Input.Measurements.Depth_Miss_Counter2 + 1;                         
                     else
                        H      = [H;Hdepth];
                        Rr     = [Rr;Rdepth];
                        dz     = [dz,d_depth];
                        Simulation.Input.Measurements.Depth_Miss_Counter2 = 0;                         
                     end
                else
                    H      = [H;Hdepth];
                    Rr     = [Rr;Rdepth];
                    dz     = [dz,d_depth];
                    Simulation.Input.Measurements.Depth_Miss_Counter2 = 0;                      
                end
        end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if strcmp(IMU_Time,depth_Time) %strcmp(IMU_Time,DVL_Time)%
                    Simulation.Input.Measurements.Depth_Counter = Simulation.Input.Measurements.Depth_Counter + 1;
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    end
    %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@    
    if include_dvl
        if I<=calib_sample && DVL_calib_count >= calib_num
            DVL_fusion_active = 1; 
            DVL_calib_count = 0;               
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            v1_dvl = Simulation.Output.INS.X_INS(1,4:6);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        elseif I > calib_sample && strcmp(IMU_Time,DVL_Time) %if strcmp(IMU_Time,depth_Time)
             DVL_fusion_active = 1; 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            v1_dvl    = zeros(1,3);
            v1_dvl(1,1) = Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,2);
            v1_dvl(1,2:3) =  Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,3:4);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end            
        if DVL_fusion_active       
            v2_dvl = v1_dvl.*(1+(SF)/100);
            vb     = C_IMU_DVL*v2_dvl';
            d_vn   = Simulation.Output.INS.X_INS(I - ave_sample + 1,4:6)-(Simulation.Output.INS.Cbn(:,:,I - ave_sample + 1)*vb)';     
            Simulation.Output.Kalman_mtx.dz_Vn (Simulation.Input.Measurements.DVL_Counter,:) = d_vn;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Vn=-Simulation.Output.INS.X_INS(I - ave_sample + 1,4:6)';
            Vn_SSM=[0      -Vn(3) Vn(2)
                    Vn(3)  0      -Vn(1)
                   -Vn(2) Vn(1)  0     ];
            Hdvl = [zeros(3),eye(3),Vn_SSM,zeros(3,6)];                
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Rdvl = [ Simulation.Output.Kalman_mtx.R.r_vx;
                     Simulation.Output.Kalman_mtx.R.r_vy;
                     Simulation.Output.Kalman_mtx.R.r_vz];                
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            S = diag(Rdvl) + Hdvl*P*Hdvl';
            Simulation.Output.Kalman_mtx.S_Vn(Simulation.Input.Measurements.DVL_Counter,:) = diag(S)';   
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Kdvl = P*Hdvl'/S;
            Simulation.Output.Kalman_mtx.K_Vn(Simulation.Input.Measurements.DVL_Counter,:) = [Kdvl(4,1),Kdvl(5,2),Kdvl(6,3)];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if Simulation.Input.Measurements.DVL_Miss_Counter2 <5 %m/s
                if (d_vn(1)^2)>TN*S(1,1) || (d_vn(2)^2)>TE*S(2,2) || (d_vn(3)^2)>TD*S(3,3)
                    Simulation.Input.Measurements.DVL_Miss_Counter = Simulation.Input.Measurements.DVL_Miss_Counter + 1;
                    Simulation.Input.Measurements.DVL_Miss_Counter2 = Simulation.Input.Measurements.DVL_Miss_Counter2 + 1;  
                    flag=1;
                else
                    H    = [ H ; Hdvl];
                    Rr   = [ Rr ; Rdvl];
                    dz   = [ dz ,d_vn];
                    Simulation.Input.Measurements.DVL_Miss_Counter2 = 0;
                end
            else
                H    = [ H ; Hdvl];
                Rr   = [ Rr ; Rdvl];
                dz   = [ dz ,d_vn];
                Simulation.Input.Measurements.DVL_Miss_Counter2 = 0;                
            end
        end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if strcmp(IMU_Time,DVL_Time)
                if flag~=1
                 Simulation.Input.Measurements.DVL2(Simulation.Input.Measurements.DVL_Counter,:)=Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,2:4);
                end
                Simulation.Input.Measurements.DVL_Counter       = Simulation.Input.Measurements.DVL_Counter + 1;
            end        
    end
    %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@       
    if include_accelrollpitch
        if Simulation.Output.Alignment.RP_active
            Simulation.Output.Alignment.RP_active = 0;
            if I<=calib_sample                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                d_AccelRoll = Simulation.Output.INS.X_INS(I - ave_sample + 1,7)-Simulation.Output.INS.X_INS(1,7);
                d_PitchRoll = Simulation.Output.INS.X_INS(I - ave_sample + 1,8)-Simulation.Output.INS.X_INS(1,8);                
            else
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                Simulation.Output.Alignment.RP_active = 0;
                d_AccelRoll = Simulation.Output.INS.X_INS(I - ave_sample + 1,7)-Simulation.Output.Alignment.ave_phi(Simulation.Output.Alignment.ave_RP_counter);
                d_PitchRoll = Simulation.Output.INS.X_INS(I - ave_sample + 1,8)-Simulation.Output.Alignment.ave_theta(Simulation.Output.Alignment.ave_RP_counter);
            end
                Simulation.Output.Kalman_mtx.dz_accelrollpitch (Simulation.Output.Alignment.ave_RP_counter,:) = [d_AccelRoll d_PitchRoll];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                C = Gamma(Simulation.Output.INS.X_INS(I - ave_sample + 1,7:9));
                i_phi=[1 0 0
                       0 1 0];
                Harp =  [zeros(2,6),-i_phi/C,zeros(2,6)];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                Rarp = [Simulation.Output.Kalman_mtx.R.r_aroll;
                        Simulation.Output.Kalman_mtx.R.r_apitch ];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                S = diag(Rarp) + Harp*P*Harp';
                Simulation.Output.Kalman_mtx.S_accelrollpitch(Simulation.Output.Alignment.ave_RP_counter,:) = diag(S)'; 
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                Karp = P*Harp'/S;
                Simulation.Output.Kalman_mtx.K_arp(Simulation.Output.Alignment.RP_counter,:) = [Karp(7,1),Karp(8,2)];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                 
                H=[H;Harp];
                Rr=[Rr;Rarp];
                dz=[dz,d_AccelRoll,d_PitchRoll];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
        end
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
    if include_heading
        if I<=calib_sample && Hdng_calib_count >= calib_num
            Hdng_fusion_active = 1;
            Hdng_calib_count = 0;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if Simulation.Output.INS.X_INS(1,9) > pi
                z_hdng = Simulation.Output.INS.X_INS(1,9) - 2*pi;
            elseif Simulation.Output.INS.X_INS(1,9) < -pi
                z_hdng = Simulation.Output.INS.X_INS(1,9) + 2*pi;
            else
                z_hdng = Simulation.Output.INS.X_INS(1,9);
            end                
            dz_hdng = Simulation.Output.INS.X_INS(I - ave_sample + 1,9) - z_hdng;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        elseif I > calib_sample && strcmp(IMU_Time,hdng_Time)
            Hdng_fusion_active = 1;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            z_hdng_c = Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,2)+ Simulation.Parameters_Misalignment.IMU_phins(3) ;
            if z_hdng_c * pi/180 > pi
                z_hdng = (z_hdng_c * pi/180) - 2*pi;
            elseif z_hdng_c * pi/180 < -pi
                z_hdng = (z_hdng_c * pi/180) + 2*pi;
            else
                z_hdng = z_hdng_c * pi/180;
            end                
            dz_hdng = Simulation.Output.INS.X_INS(I - ave_sample + 1,9) - z_hdng;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end        
        if Hdng_fusion_active
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if dz_hdng> pi
                dz_hdng = dz_hdng - 2*pi;
            end
            if dz_hdng< -pi
                dz_hdng = dz_hdng + 2*pi;
            end
            Simulation.Output.Kalman_mtx.dz_heading (Simulation.Input.Measurements.hdng_Counter,:) = dz_hdng;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            C = Gamma(Simulation.Output.INS.X_INS(I - ave_sample + 1,7:9));
            i_psi=[0 0 1];     
            Hhding = [zeros(1,6),-i_psi/C,zeros(1,6)];               
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Rhding = Simulation.Output.Kalman_mtx.R.r_yaw;                       
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            S = Rhding + Hhding*P*Hhding';
            Simulation.Output.Kalman_mtx.S_heading(Simulation.Input.Measurements.hdng_Counter,:) = diag(S)'; 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Khding = P*Hhding'/S;
            Simulation.Output.Kalman_mtx.K_hdng(Simulation.Input.Measurements.hdng_Counter,:) = Khding(9,1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
            if (dz_hdng^2)> Th*S && Simulation.Input.Measurements.Hdng_Miss_Counter2 < 3%rad 
            	Simulation.Input.Measurements.Hdng_Miss_Counter  = Simulation.Input.Measurements.Hdng_Miss_Counter + 1;
                Simulation.Input.Measurements.Hdng_Miss_Counter2 = Simulation.Input.Measurements.Hdng_Miss_Counter2 + 1;
            else
                H  = [ H  ; Hhding];
                Rr = [ Rr ; Rhding];
                dz = [ dz , dz_hdng];
                    
                Simulation.Input.Measurements.Hdng_Miss_Counter2 =0;
            end

        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if strcmp(IMU_Time,hdng_Time)
                Simulation.Input.Measurements.hdng_Counter  = Simulation.Input.Measurements.hdng_Counter + 1;
            end        
    end 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    R=diag(Rr);%measurement noise covariance matrix
    Simulation.Output.Kalman_mtx.H        = 0;
    Simulation.Output.Kalman_mtx.R.Rmatrx = 0;
    Simulation.Output.Kalman_mtx.dz       = 0;
    Simulation.Output.Kalman_mtx.H        = H;%measurement matrix
    Simulation.Output.Kalman_mtx.R.Rmatrx = R;
    Simulation.Output.Kalman_mtx.dz       = dz';
    %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@    
end    