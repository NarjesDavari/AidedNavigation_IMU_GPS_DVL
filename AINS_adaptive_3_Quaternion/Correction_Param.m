%Computation of The measurement model matrix(H) and the measurement error
%variance(R) in indirect approch (ESKF).
%for LSTS
function [ Simulation,V,flag_accelrollpitch,alphaN,betaN ] = Correction_Param_2(Simulation,Selection_Param,SF ,I,C_DVL_IMU,P,Misalignment_IMU_phins,ave_sample,calib_sample,alpha,beta) 
WithoutAdaptive=Simulation.select_adaptive.WithoutAdaptive;
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

global GPS_fusion_active ;
global depth_fusion_active ;
global DVL_fusion_active ;
global Hdng_fusion_active ;
global R_DVL_active;
global rollpitch_fusion_active;
global Updt_Cntr;
GPS_fusion_active   = 0;
depth_fusion_active = 0;
DVL_fusion_active   = 0;
Hdng_fusion_active  = 0;
R_DVL_active         =0;
rollpitch_fusion_active =0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
IMU_Time               = Selection_Param{1};
GPS_Time               = Selection_Param{2};
depth_Time             = Selection_Param{3};
DVL_Time               = Selection_Param{4};
hdng_Time              = Selection_Param{5};
include_GPS            = Selection_Param{6};
include_depthmeter     = Selection_Param{7};
include_dvl            = Selection_Param{8};
include_heading        = Selection_Param{9};
include_rollpitch      = 0;
include_accelrollpitch = Selection_Param{10};
mu                     = Selection_Param{11};
H=[];
M=[];
H2=[];
H1=[];
Rr=[];
dz=[];
V=[];
a=0;
b=0;
flag=0;
flag_accelrollpitch=0;
Dlength = length(Simulation.Input.Measurements.IMU);

T_Lat = Simulation.Rej_Cof.Lat;
T_lon = Simulation.Rej_Cof.Lon;
Tz    = Simulation.Rej_Cof.Z;

TN = Simulation.Rej_Cof.VN;
TE = Simulation.Rej_Cof.VE;
TD = Simulation.Rej_Cof.VD;
Th = Simulation.Rej_Cof.H;

Simulation.Output.Kalman_mtx.Rej_CofDVL=[TN TE TD];

alphaN=[];
betaN=[];
H_N=[];
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
            H2gps = [ 1,0,0,0,0,0,0,0,0 
                      0,1,0,0,0,0,0,0,0];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if WithoutAdaptive || Updt_Cntr==0
               Rgps = [ Simulation.Output.Kalman_mtx.R.r_Lat;
                        Simulation.Output.Kalman_mtx.R.r_lon]; 
            else
            Rgps = [Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter,1);
                     Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(Simulation.Input.Measurements.GPS_Counter,2)];
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            S = diag(Rgps) + Hgps*P*Hgps';
            Simulation.Output.Kalman_mtx.S_gps(Simulation.Input.Measurements.GPS_Counter,:) = diag(S)';
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Kgps = P*Hgps'/S;
%             Simulation.Output.Kalman_mtx.K_gps(Simulation.Input.Measurements.GPS_Counter,:) = [Kgps(1,1),Kgps(2,2)];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if Simulation.Input.Measurements.GPS_Miss_Counter2 < 3
                if (d_GPS(1)^2)>T_Lat*S(1,1) || (d_GPS(2)^2)>T_lon*S(2,2)
                    Simulation.Input.Measurements.GPS_Miss_Counter = Simulation.Input.Measurements.GPS_Miss_Counter + 1;
                    Simulation.Input.Measurements.GPS_Miss_Counter2 = Simulation.Input.Measurements.GPS_Miss_Counter2 + 1;
                else
                    H    = [ H ; Hgps ];
                    Rr   = [Rr; Rgps];
                    dz   = [dz,d_GPS];
                    H_N=[H_N;H2gps];
                    alphaN=[alphaN;alpha(1:2)];
                    betaN=[betaN;beta(1:2)];
                    Simulation.Input.Measurements.GPS_Miss_Counter2 = 0;
                 end
            else
                H    = [ H ; Hgps ];
                Rr   = [Rr; Rgps];
                dz   = [dz,d_GPS];
                H_N=[H_N;H2gps];
                alphaN=[alphaN;alpha(1:2)];
                betaN=[betaN;beta(1:2)];
                Simulation.Input.Measurements.GPS_Miss_Counter2 = 0;              
            end
            V=[V,d_GPS];
            H1=[H1,1 1];
            H2=[H2,0 0];
        else
            V = [V,0 0];
            H1=[H1,0 0];
            H2=[H2,1 1];
        end
        if  str2double(GPS_Time)<=str2double(IMU_Time)
            Simulation.Input.Measurements.GPS_Counter   = Simulation.Input.Measurements.GPS_Counter + 1;
        end
   else
      V=[V,0 0];
      H1=[H1,0 0];
      H2=[H2,1 1];
    end
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
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
                H2depth = [0,0,1,0,0,0,0,0,0];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if WithoutAdaptive || Updt_Cntr==0
                    Rdepth = Simulation.Output.Kalman_mtx.R.r_alt;   
                else
                    Rdepth = Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(Simulation.Input.Measurements.Depth_Counter,1);    
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
                S = Rdepth + Hdepth*P*Hdepth';
                Simulation.Output.Kalman_mtx.S_depth(Simulation.Input.Measurements.Depth_Counter,:) = diag(S)';                  
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 Kdepth = P*Hdepth'/S;
%                 Simulation.Output.Kalman_mtx.K_depth(Simulation.Input.Measurements.Depth_Counter,:) = Kdepth(3,1);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if Simulation.Input.Measurements.Depth_Miss_Counter2 < 3
                     if (d_depth^2)>Tz*S
                        Simulation.Input.Measurements.Depth_Miss_Counter  = Simulation.Input.Measurements.Depth_Miss_Counter + 1;
                        Simulation.Input.Measurements.Depth_Miss_Counter2 = Simulation.Input.Measurements.Depth_Miss_Counter2 + 1;                         
                     else
                        H      = [H;Hdepth];
                        Rr     = [Rr;Rdepth];
                        dz     = [dz,d_depth];
                        H_N=[H_N;H2depth];
                        alphaN=[alphaN;alpha(3)];
                        betaN=[betaN;beta(3)];
                        Simulation.Input.Measurements.Depth_Miss_Counter2 = 0;                         
                     end
                else
                    H      = [H;Hdepth];
                    Rr     = [Rr;Rdepth];
                    dz     = [dz,d_depth];
                    H_N=[H_N;H2depth];
                    alphaN=[alphaN;alpha(3)];
                    betaN=[betaN;beta(3)];
                    Simulation.Input.Measurements.Depth_Miss_Counter2 = 0;
                end
                V=[V,d_depth];
                H1=[H1,1];
                H2=[H2,0];
        else
            V = [V,0];
            H1=[H1,0];
            H2=[H2,1];
        end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if str2double(depth_Time)<=str2double(IMU_Time) %strcmp(IMU_Time,depth_Time)%
                    Simulation.Input.Measurements.Depth_Counter = Simulation.Input.Measurements.Depth_Counter + 1;
                end 
    else
       V = [V,0];
       H1=[H1,0];
       H2=[H2,1];
    end
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
            v1_dvl(1,2:3) = -Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,3:4);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end            
        if DVL_fusion_active       
            v2_dvl = v1_dvl.*(1+(SF)/100);
            vb     = C_DVL_IMU*v2_dvl';
            d_vn   = Simulation.Output.INS.X_INS(I - ave_sample + 1,4:6)-(Simulation.Output.INS.Cbn(:,:,I - ave_sample + 1)*vb)';     
            Simulation.Output.Kalman_mtx.dz_Vn (Simulation.Input.Measurements.DVL_Counter,:) = d_vn;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Vn=-Simulation.Output.INS.X_INS(I - ave_sample + 1,4:6)';
            Vn_SSM=[0      -Vn(3) Vn(2)
                    Vn(3)  0      -Vn(1)
                   -Vn(2) Vn(1)  0     ];
            Hdvl = [zeros(3),eye(3),Vn_SSM,zeros(3,6)];   
            H2dvl = [zeros(3),eye(3),zeros(3)];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if WithoutAdaptive || Updt_Cntr==0
                 Rdvl = [ Simulation.Output.Kalman_mtx.R.r_vx;
                          Simulation.Output.Kalman_mtx.R.r_vy;
                          Simulation.Output.Kalman_mtx.R.r_vz]; 
            else
            Rdvl = [ Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter,1);
                     Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter,2);
                     Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(Simulation.Input.Measurements.DVL_Counter,3)];   
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            S = diag(Rdvl) + Hdvl*P*Hdvl';
%             [u,s,v]=svd(S);
%             S=u*sqrt(s)*u';
             Simulation.Output.Kalman_mtx.S_Vn(Simulation.Input.Measurements.DVL_Counter,:) = diag(S)';   
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Kdvl = P*Hdvl'/S;
%             Simulation.Output.Kalman_mtx.K_Vn(Simulation.Input.Measurements.DVL_Counter,:) = [Kdvl(4,1),Kdvl(5,2),Kdvl(6,3)];
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if Simulation.Input.Measurements.DVL_Miss_Counter2 < 3 %m/s
                if (d_vn(1)^2)>TN*S(1,1) || (d_vn(2)^2)>TE*S(2,2) || (d_vn(3)^2)>TD*S(3,3)
                    Simulation.Input.Measurements.DVL_Miss_Counter = Simulation.Input.Measurements.DVL_Miss_Counter + 1;
                    Simulation.Input.Measurements.DVL_Miss_Counter2 = Simulation.Input.Measurements.DVL_Miss_Counter2 + 1;  
                    flag=1;
                else
                    H    = [ H ; Hdvl];
                    Rr   = [ Rr ; Rdvl];
                    dz   = [ dz ,d_vn];
                    H_N=[H_N;H2dvl];
                    alphaN=[alphaN;alpha(4:6)];
                    betaN=[betaN;beta(4:6)];
                    Simulation.Input.Measurements.DVL_Miss_Counter2 = 0;
                    R_DVL_active=1;
                end
            else
                H    = [ H ; Hdvl];
                Rr   = [ Rr ; Rdvl];
                dz   = [ dz ,d_vn];
                H_N=[H_N;H2dvl];
                alphaN=[alphaN;alpha(4:6)];
                betaN=[betaN;beta(4:6)];
                Simulation.Input.Measurements.DVL_Miss_Counter2 = 0; 
                R_DVL_active=1;
            end
             V= [V,d_vn];
             H1=[H1,1 1 1];
             H2=[H2,0 0 0];
        else
           V = [V,0 0 0];
           H1=[H1,0 0 0];
           H2=[H2,1 1 1];
        end
        if  str2double(DVL_Time)<=str2double(IMU_Time) %%strcmp(IMU_Time,DVL_Time)
            if flag~=1
                 Simulation.Input.Measurements.DVL2(Simulation.Input.Measurements.DVL_Counter,:)=Simulation.Input.Measurements.DVL(Simulation.Input.Measurements.DVL_Counter,2:4);
            end
                Simulation.Input.Measurements.DVL_Counter       = Simulation.Input.Measurements.DVL_Counter + 1;
        end
  else
      V= [V,0 0 0];
      H1=[H1,0 0 0];
      H2=[H2,1 1 1];
  end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if include_rollpitch
            if strcmp(IMU_Time,incln_Time)
                rollpitch_fusion_active =1;
                a=1;
                C = Gamma(Simulation.Output.INS.X_INS(I - ave_sample + 1,7:9));
                i_phi=[1 0 0
                       0 1 0];
                H=[H;
                    zeros(2,6),-i_phi/C,zeros(2,6)];
                Rr=[Rr;
                    Simulation.Output.Kalman_mtx.R.r_roll;
                    Simulation.Output.Kalman_mtx.R.r_pitch];   
                   
                d_roll=Simulation.Output.INS.X_INS(I - ave_sample + 1,7)-Simulation.Input.Measurements.RollPitch(Simulation.Input.Measurements.incln_Counter,2,i)*pi/180;
                d_pitch=Simulation.Output.INS.X_INS(I - ave_sample + 1,8)-Simulation.Input.Measurements.RollPitch(Simulation.Input.Measurements.incln_Counter,3,i)*pi/180;
                dz=[dz,d_roll,d_pitch];
                V=[V,d_roll,d_pitch];  
                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                   Simulation.Input.Measurements.incln_Counter = Simulation.Input.Measurements.incln_Counter + 1;
            end
    end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

                b=1;
                flag_accelrollpitch=1;
                C = Gamma(Simulation.Output.INS.X_INS(I - ave_sample + 1,7:9));
                i_phi=[1 0 0
                       0 1 0];
                Harp =  [zeros(2,6),-i_phi/C,zeros(2,6)];
                H2arp =  [zeros(2,6),i_phi];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if WithoutAdaptive || Updt_Cntr==0
                  Rarp = [Simulation.Output.Kalman_mtx.R.r_aroll;
                          Simulation.Output.Kalman_mtx.R.r_apitch ];
                else
                Rarp = [Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter,1);
                        Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(Simulation.Input.Measurements.accelrollpitch_Counter,1) ];
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                S = diag(Rarp) + Harp*P*Harp';
                Simulation.Output.Kalman_mtx.S_accelrollpitch(Simulation.Output.Alignment.ave_RP_counter,:) = diag(S)'; 
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 Karp = P*Harp'/S;
%                 Simulation.Output.Kalman_mtx.K_arp(Simulation.Output.Alignment.RP_counter,:) = [Karp(7,1),Karp(8,2)];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                 
                H=[H;Harp];
                Rr=[Rr;Rarp];
                dz=[dz,d_AccelRoll,d_PitchRoll];
                H_N=[H_N;H2arp];
                alphaN=[alphaN;alpha(7:8)];
                betaN=[betaN;beta(7:8)];
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                
                 V=[V,d_AccelRoll,d_PitchRoll];
                   Simulation.Input.Measurements.accelrollpitch_Counter   = Simulation.Input.Measurements.accelrollpitch_Counter + 1;
                 H1=[H1,1 1];
                 H2=[H2,0 0];

            end
    end
    if b~=1  && a~=1
       V = [V,0 0];
       H1=[H1,0 0];
       H2=[H2,1 1];
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
            z_hdng_c = Simulation.Input.Measurements.Heading(Simulation.Input.Measurements.hdng_Counter,2)+ Misalignment_IMU_phins(3) ;
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
            H2hding = [zeros(1,8),1];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if WithoutAdaptive || Updt_Cntr==0
                Rhding = Simulation.Output.Kalman_mtx.R.r_yaw;  
            else
            Rhding = Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(Simulation.Input.Measurements.hdng_Counter,1);   
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            S = Rhding + Hhding*P*Hhding';
            Simulation.Output.Kalman_mtx.S_heading(Simulation.Input.Measurements.hdng_Counter,:) = diag(S)'; 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Khding = P*Hhding'/S;
%             Simulation.Output.Kalman_mtx.K_hdng(Simulation.Input.Measurements.hdng_Counter,:) = Khding(9,1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
            if (dz_hdng^2)> Th*S && Simulation.Input.Measurements.Hdng_Miss_Counter2 < 3%rad 
            	Simulation.Input.Measurements.Hdng_Miss_Counter  = Simulation.Input.Measurements.Hdng_Miss_Counter + 1;
                Simulation.Input.Measurements.Hdng_Miss_Counter2 = Simulation.Input.Measurements.Hdng_Miss_Counter2 + 1;
            else
                H  = [ H  ; Hhding];
                Rr = [ Rr ; Rhding];
                dz = [ dz , dz_hdng];
                H_N=[H_N;H2hding];
                alphaN=[alphaN;alpha(9)];
                betaN=[betaN;beta(9)];
                Simulation.Input.Measurements.Hdng_Miss_Counter2 =0;
            end
                V=[V,dz_hdng,zeros(1,6)];
                H1=[H1,1];
                H2=[H2,0];
        else
            V = [V,zeros(1,7)];
            H1=[H1,0];
            H2=[H2,1];
        end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if  str2double(hdng_Time)<=str2double(IMU_Time) %%strcmp(IMU_Time,hdng_Time)
                Simulation.Input.Measurements.hdng_Counter  = Simulation.Input.Measurements.hdng_Counter + 1;
            end
    else
      V=[V,zeros(1,7)];
      H1=[H1,0];
      H2=[H2,1];
    end 
 M= [eye(3),zeros(3,6);zeros(3),Simulation.Output.INS.Cbn(:,:,I - ave_sample + 1),zeros(3);zeros(3,6),eye(3)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    R=diag(Rr);%measurement noise covariance matrix
    Simulation.Output.Kalman_mtx.H=0;
    Simulation.Output.Kalman_mtx.H1=0;
    Simulation.Output.Kalman_mtx.H2=0;
    Simulation.Output.Kalman_mtx.M=0;
    Simulation.Output.Kalman_mtx.R.Rmatrx=0;
    Simulation.Output.Kalman_mtx.dz=0;
%     Simulation.Output.Kalman_mtx.V=0;
    Simulation.Output.Kalman_mtx.H=H;%measurement matrix
    Simulation.Output.Kalman_mtx.H1=diag(H1);
    Simulation.Output.Kalman_mtx.H2=diag(H2);
    Simulation.Output.Kalman_mtx.M=M;
    Simulation.Output.Kalman_mtx.R.Rmatrx=R; 
    Simulation.Output.Kalman_mtx.dz=dz';
    Simulation.Output.Kalman_mtx.H_N=H_N;
%     Simulation.Output.Kalman_mtx.V(I,:)=V;
end
%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
