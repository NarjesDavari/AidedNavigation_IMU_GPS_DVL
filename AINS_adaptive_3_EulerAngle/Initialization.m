%Initialization of the orientation, transformation matrix, vvelocity,
%position, acceleration, and so on.
function [ Simulation,flag_Qadapt ] = Initialization( Simulation , fs ,Include_Q_adaptive,ave_sample)
            global Updt_Cntr;
            global Cbn_det;
            Updt_Cntr = 0;     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Constant Parameters
    R = 6378137;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            Dlength = length(Simulation.Input.Measurements.IMU);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Input.Measurements.GPS_Counter   = 1; %GPS Counter
            Simulation.Input.Measurements.DVL_Counter   = 1; %DVL Counter%Path4:1658
            Simulation.Input.Measurements.Depth_Counter = 1; %Depth Counter
            Simulation.Input.Measurements.hdng_Counter  = 1; %Heading Counter
            Simulation.Input.Measurements.incln_Counter = 1; %Inclinometer Counter
            Simulation.Input.Measurements.accelrollpitch_Counter=1;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Input.Measurements.GPS_Counter_fusion = 1;
            Simulation.Input.Measurements.Depth_Counter_fusion = 1;
             Simulation.Input.Measurements.DVL_Counter_fusion = 1;
              Simulation.Input.Measurements.hdng_Counter_fusion = 1;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Dlength = length(Simulation.Input.Measurements.IMU);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Cbn_det   = zeros(Dlength,1);  
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           
            %Initial acceleration and angular velocity
%             Bias1g =Simulation.Input.InitialParam.Initialbiasg;
            W1ib_b(1)=Simulation.Input.Measurements.IMU(1,5,1);    
            W1ib_b(2)=Simulation.Input.Measurements.IMU(1,6,1);    
            W1ib_b(3)=Simulation.Input.Measurements.IMU(1,7,1);
                   
%             Bias1a =Simulation.Input.InitialParam.Initialbiasa; 
            f1b(1)=Simulation.Input.Measurements.IMU(1,2,1);
            f1b(2)=Simulation.Input.Measurements.IMU(1,3,1);
            f1b(3)=Simulation.Input.Measurements.IMU(1,4,1);                        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%Alignment:computation of initial transformation matrix
            [ Simulation , gl , ave_fb , ave_W ] = Coarse_Alignment_2( Simulation,ave_sample  );
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Euler1                                      = Simulation.Output.INS.X_INS(1,7:9);
            Simulation.Input.InitialParam.InitialEuler  = Euler1;
            Cbn_INS                                     = InCBN(Euler1);
            Simulation.Output.INS.Cbn                   = zeros(3,3,Dlength-ave_sample +1 );
            Simulation.Output.INS.Cbn(:,:,1)            = Cbn_INS;
            Simulation.Output.ESKF.Cbn_corrected        = zeros(3,3,Dlength-ave_sample +1 );
            Simulation.Output.ESKF.Cbn_corrected(:,:,1) = Cbn_INS;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Input.InitialParam.InitialVelocity            = Simulation.Output.INS.X_INS(1,4:6);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.INS.FilteredSignal.filtered_RP         = zeros(Dlength-ave_sample +1 ,3);
            Simulation.Output.INS.FilteredSignal.filtered_RP(1,:)    = ave_fb;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.INS.FilteredSignal.filtered_Accel      = zeros(Dlength-ave_sample +1 ,3);
            Simulation.Output.INS.FilteredSignal.filtered_Accel(1,:) = ave_fb;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.INS.FilteredSignal.filtered_Gyro       = zeros(Dlength-ave_sample +1 ,3);
            Simulation.Output.INS.FilteredSignal.filtered_Gyro(1,:)  = ave_W;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.INS.norm.Accel_norm    = zeros(Dlength-ave_sample +1 ,1);
            Simulation.Output.INS.norm.Gyro_norm     = zeros(Dlength-ave_sample +1 ,1);
            Simulation.Output.INS.norm.Accel_norm (1,1) = abs(norm(ave_fb) -  norm(gl));
            Simulation.Output.INS.norm.Gyro_norm (1,1) = norm(ave_W);      
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.Alignment.phi              = zeros(Dlength-ave_sample +1 ,1);
            Simulation.Output.Alignment.theta            = zeros(Dlength-ave_sample +1 ,1);
            Simulation.Output.Alignment.ave_phi          = zeros(fix(Dlength-ave_sample +1 /1),1);
            Simulation.Output.Alignment.ave_theta        = zeros(fix(Dlength-ave_sample +1 /1),1);            
%             Simulation.Output.Alignment.time         = zeros(Dlength-ave_sample +1 ,1);
            Simulation.Output.Alignment.RP_counter       = 1;
            Simulation.Output.Alignment.RP_counter2      = 0;
            Simulation.Output.Alignment.ave_RP_counter   = 1;
            Simulation.Output.Alignment.RP_active        = 0;
            
            Simulation.Output.Alignment.theta(1,1)       = Simulation.Output.INS.X_INS(1,8);
            Simulation.Output.Alignment.phi(1,1)         = Simulation.Output.INS.X_INS(1,7);
            Simulation.Output.Alignment.ave_theta(1,1)   = Simulation.Output.INS.X_INS(1,8);
            Simulation.Output.Alignment.ave_phi(1,1)     = Simulation.Output.INS.X_INS(1,7);           
%             Simulation.Output.Alignment.time         = Simulation.Input.Measurements.IMU(1,1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            W_Coriolis = Coriolis_correction( Simulation.Output.INS.X_INS(1,1:6)  );
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.INS.fnn      = zeros(Dlength-ave_sample ,3);
            f1nn                           = (Cbn_INS*ave_fb')';%%convert initial Accelerometer frome  body to navigation
            %Simulation.Output.INS.fnn(1,:) = f1nn;
            f1n                            = f1nn - cross(W_Coriolis,Simulation.Output.INS.X_INS(1,4:6)) + gl;
            Simulation.Output.INS.fn       = zeros(Dlength-ave_sample +1 ,3);
            Simulation.Output.INS.fn(1,:)  = f1n;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
            %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
            %error states                
            dX1                            = (zeros(1,15))';%column vector
            Simulation.Output.ESKF.dX      = zeros(Dlength-ave_sample +1 ,size(dX1,1));
            Simulation.Output.ESKF.dX(1,:) = dX1';        
            %&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
            %O_corrected:corrected ouyput(position,veloccity and euler angles)
            Simulation.Output.ESKF.O_corrected      = zeros(Dlength-ave_sample +1 ,size(dX1,1));
            Simulation.Output.ESKF.O_corrected(1,:)=Simulation.Output.INS.X_INS(1,1:15);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.Kalman_mtx.Q_diag = zeros(Dlength-ave_sample ,size(dX1,1));
            Simulation.Output.Kalman_mtx.F_diag = zeros(Dlength-ave_sample ,size(dX1,1));
            Simulation.Output.Kalman_mtx.G_diag = zeros(Dlength-ave_sample ,24);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
            Simulation.Output.Kalman_mtx.dz_gps              = zeros(Dlength-ave_sample +1 ,2);
            Simulation.Output.Kalman_mtx.dz_depth            = zeros(Dlength-ave_sample +1 ,1);
            Simulation.Output.Kalman_mtx.dz_Vn               = zeros(Dlength-ave_sample +1 ,3);
            Simulation.Output.Kalman_mtx.dz_rollpitch        = zeros(Dlength-ave_sample +1 ,2);
            Simulation.Output.Kalman_mtx.dz_accelrollpitch   = zeros(Dlength-ave_sample +1 ,2);
            Simulation.Output.Kalman_mtx.dz_heading          = zeros(Dlength-ave_sample +1 ,1);
            
            Simulation.Output.Kalman_mtx.S_gps               = zeros(Dlength-ave_sample +1 ,2);
            Simulation.Output.Kalman_mtx.S_depth             = zeros(Dlength-ave_sample +1 ,1);
            Simulation.Output.Kalman_mtx.S_Vn                = zeros(Dlength-ave_sample +1 ,3);
            Simulation.Output.Kalman_mtx.S_rollpitch         = zeros(Dlength-ave_sample +1 ,2);
            Simulation.Output.Kalman_mtx.S_accelrollpitch    = zeros(Dlength-ave_sample +1 ,2);
            Simulation.Output.Kalman_mtx.S_heading           = zeros(Dlength-ave_sample +1 ,1); 
            
            Simulation.Input.Measurements.GPS_Miss_Counter   = 0;
            Simulation.Input.Measurements.Depth_Miss_Counter = 0;
            Simulation.Input.Measurements.DVL_Miss_Counter   = 0;
            Simulation.Input.Measurements.Hdng_Miss_Counter  = 0;
            
            Simulation.Input.Measurements.GPS_Miss_Counter2   = 0;
            Simulation.Input.Measurements.Depth_Miss_Counter2 = 0;
            Simulation.Input.Measurements.DVL_Miss_Counter2   = 0;
            Simulation.Input.Measurements.Hdng_Miss_Counter2  = 0;      
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.Kalman_mtx.K_gps            = zeros(Dlength-ave_sample +1 ,2);
            Simulation.Output.Kalman_mtx.K_depth          = zeros(Dlength-ave_sample +1 ,1);
            Simulation.Output.Kalman_mtx.K_Vn             = zeros(Dlength-ave_sample +1 ,3);
            Simulation.Output.Kalman_mtx.K_arp            = zeros(Dlength-ave_sample +1 ,2);
            Simulation.Output.Kalman_mtx.K_hdng           = zeros(Dlength-ave_sample +1 ,1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.Kalman_mtx.adaptive_innovation=0;
            Simulation.Output.Kalman_mtx.adaptive_residual=0;
            Simulation.Output.Kalman_mtx.update_counter =1;
            Simulation.Output.Kalman_mtx.Number_adaptive=0;                                                
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Simulation.Output.Kalman_mtx.Q_diag=zeros(Dlength-ave_sample,size(dX1,1));
            Simulation.Output.Kalman_mtx.norm_Q=zeros(Dlength-ave_sample,1);
            Simulation.Output.Kalman_mtx.tun=zeros(Dlength-ave_sample,1);
            %corrected position,velocity and accel(in navigation frame)
            x1=[Simulation.Output.INS.X_INS(1,1:6),Simulation.Output.INS.fnn(1,:)];
            C1=Simulation.Output.INS.Cbn(:,:,1);
            [Simulation,flag_Qadapt]=AQ_calcul(x1,C1,Simulation,fs,1,Include_Q_adaptive,0);
                                         
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Simulation.Output.User_Def_Sim.Kalman_mtx.P = diag([1,0.01,0.01,0.01,0.018773,0.018773,0.018773]);

            Simulation.Output.Kalman_mtx.P = zeros(15,15);
            %%%%%%% for VB and ESKF
            b1=0.01;
            b2=1000;
            b31=100;b32=100;b33=1;
            b41=0.01;b42=0.01;b43=1;
            b5=0.01;
            b61=.1;b62=.1;b63=.1;
            
            %%%%%%%%%%%%%%%%%%%%% check for Hinf+ VB
%             b1=1; %%1e-3;
%             b2=1; %%1e-3;
%             b31=1; %%1e-3;
%             b32=1; %%1e-3;
%             b33=1; %%1e-3;
%             b41=1; %%1e-3;
%             b42=1; %%1e-3;
%             b43=1; %%1e-3;
%             b5=1; %%1e-3;
%             b61=1; %%1e-3;
%             b62=1; %%1e-3;
%             b63=1; %%1e-3;
            %%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.Kalman_mtx.P(1,1) = 1e-6*b1;
            Simulation.Output.Kalman_mtx.P(2,2) = 1e-6*b1;
            
            Simulation.Output.Kalman_mtx.P(3,3) = 1e-6*b2;
            
            Simulation.Output.Kalman_mtx.P(4,4) = 1e-6*b31;
            Simulation.Output.Kalman_mtx.P(5,5) = 1e-6*b32;
            Simulation.Output.Kalman_mtx.P(6,6) = 1e-6*b33;
            
            Simulation.Output.Kalman_mtx.P(7,7) =  1e-6*b41;
            Simulation.Output.Kalman_mtx.P(8,8) = 1e-6*b42;
            Simulation.Output.Kalman_mtx.P(9,9) =  1e-6*b43;
            
            Simulation.Output.Kalman_mtx.P(10,10) = 1e-6*b5;
            Simulation.Output.Kalman_mtx.P(11,11) = 1e-6*b5;
            Simulation.Output.Kalman_mtx.P(12,12) =  1e-6*b5;
            
            Simulation.Output.Kalman_mtx.P(13,13) =  1e-6*b61;
            Simulation.Output.Kalman_mtx.P(14,14) =  1e-6*b62;
            Simulation.Output.Kalman_mtx.P(15,15) =  1e-6*b63;
            Simulation.Output.Kalman_mtx.P0_Bias=[Simulation.Output.Kalman_mtx.P(10,10) Simulation.Output.Kalman_mtx.P(11,11) Simulation.Output.Kalman_mtx.P(12,12)...
               Simulation.Output.Kalman_mtx.P(13,13) Simulation.Output.Kalman_mtx.P(14,14)  Simulation.Output.Kalman_mtx.P(15,15)];
            
            Simulation.Output.Kalman_mtx.P_diag=zeros(Dlength,size(dX1,1));
            Simulation.Output.Kalman_mtx.P_diag(1,:)=[Simulation.Output.Kalman_mtx.P(1,1) Simulation.Output.Kalman_mtx.P(2,2) Simulation.Output.Kalman_mtx.P(3,3)...
                                                      Simulation.Output.Kalman_mtx.P(4,4) Simulation.Output.Kalman_mtx.P(5,5) Simulation.Output.Kalman_mtx.P(6,6)...
                                                      Simulation.Output.Kalman_mtx.P(7,7) Simulation.Output.Kalman_mtx.P(8,8) Simulation.Output.Kalman_mtx.P(9,9)...
                                                      Simulation.Output.Kalman_mtx.P(10,10) Simulation.Output.Kalman_mtx.P(11,11) Simulation.Output.Kalman_mtx.P(12,12)...
                                                      Simulation.Output.Kalman_mtx.P(13,13) Simulation.Output.Kalman_mtx.P(14,14)  Simulation.Output.Kalman_mtx.P(15,15)];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%             Simulation.Output.Kalman_mtx.Q_adaptive=Simulation.Output.Kalman_mtx.Q;
%             Simulation.Output.Kalman_mtx.V=zeros(Dlength-ave_sample +1,15);
            Simulation.Output.INS.Wnb_b=zeros(Dlength,3);

            Simulation.Output.Kalman_mtx.update_adaptive_counter=0;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             Simulation.Output.Kalman_mtx.R_adaptive.R_GPS=zeros(length(Simulation.Input.Measurements.GPS),2);
            Simulation.Input.Measurements.GPS_Counter_R=1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(1:Simulation.Input.Measurements.GPS_Counter,1)=Simulation.Output.Kalman_mtx.R.r_Lat;
            Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(1:Simulation.Input.Measurements.GPS_Counter,2)=Simulation.Output.Kalman_mtx.R.r_lon;
            
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch=zeros(length(Simulation.Input.Measurements.IMU)-ave_sample +1,2);
            Simulation.Input.Measurements.accelrollpitch_Counter_R=1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(1:Simulation.Input.Measurements.accelrollpitch_Counter,1)=Simulation.Output.Kalman_mtx.R.r_aroll;
            Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(1:Simulation.Input.Measurements.accelrollpitch_Counter,2)=Simulation.Output.Kalman_mtx.R.r_apitch;
             
            Simulation.Output.Kalman_mtx.R_adaptive.R_Depth = zeros(length(Simulation.Input.Measurements.Depth),1);
            Simulation.Input.Measurements.Depth_Counter_R=1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(1:Simulation.Input.Measurements.Depth_Counter,1) =Simulation.Output.Kalman_mtx.R.r_alt;
             
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL = zeros(length(Simulation.Input.Measurements.DVL),3);
            Simulation.Input.Measurements.DVL_Counter_R=1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(1:Simulation.Input.Measurements.DVL_Counter,1) =Simulation.Output.Kalman_mtx.R.r_vx;
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(1:Simulation.Input.Measurements.DVL_Counter,2) =Simulation.Output.Kalman_mtx.R.r_vy;
            Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(1:Simulation.Input.Measurements.DVL_Counter,3) =Simulation.Output.Kalman_mtx.R.r_vz;
            
%             Simulation.Output.Kalman_mtx.R_adaptive.R_rollpitch = zeros(length(Simulation.Input.Measurements.RollPitch),2);
            Simulation.Output.Kalman_mtx.R_adaptive.R_Heading = zeros(length(Simulation.Input.Measurements.Heading),1);
            Simulation.Input.Measurements.hdng_Counter_R=1;
            Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(1:Simulation.Input.Measurements.hdng_Counter,1) =Simulation.Output.Kalman_mtx.R.r_yaw;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.ESKF.Correction_Counter = 0;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Variational Bayesian
            
            alpha_VB=[1,1,1,1,1,1,1,1,1];
            alpha_VB=Simulation.parameter_VB.alpha*alpha_VB;%%%Simulation.Output.Kalman_mtx.alpha_VB*ones(1,9); %%%
            beta_VB=[Simulation.Output.Kalman_mtx.R.r_Lat,Simulation.Output.Kalman_mtx.R.r_lon,Simulation.Output.Kalman_mtx.R.r_alt,...
                Simulation.Output.Kalman_mtx.R.r_vx,Simulation.Output.Kalman_mtx.R.r_vy,Simulation.Output.Kalman_mtx.R.r_vz,...
                Simulation.Output.Kalman_mtx.R.r_aroll,Simulation.Output.Kalman_mtx.R.r_apitch,Simulation.Output.Kalman_mtx.R.r_yaw];%%./alpha_VB;
             
             Simulation.Output.Kalman_mtx.alpha_VB=(alpha_VB)';
             Simulation.Output.Kalman_mtx.alpha_VB_init=Simulation.Output.Kalman_mtx.alpha_VB;
             Simulation.Output.Kalman_mtx.beta_VB_init=(beta_VB)';
             Simulation.Output.Kalman_mtx.beta_VB=(beta_VB)';   
             rho=[0.2,0.2,0.2,0.6,0.6,0.6,0.8,0.8,0.8];
             Simulation.Output.Kalman_mtx.rho_VB=Simulation.parameter_VB.rho;
             
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             %% parameters of VB_adaptive Q and R
            m=2; %%%size of Y
            n = 15; %%% size of X
            coeff_U =10;
            update_counter=Simulation.Output.Kalman_mtx.update_counter;
            tau = Simulation.parameter_VB_adaptiveQR.tau;
            Simulation.Output.Kalman_mtx.VB_adaptiveQR.u_hat(update_counter,1) = (m+1+tau); %%Simulation.parameter_VB_adaptiveQR.u0;
            R_GPS =diag([Simulation.Output.Kalman_mtx.R.r_Lat,Simulation.Output.Kalman_mtx.R.r_lon]);
            R_depth=Simulation.Output.Kalman_mtx.R.r_alt;
            R_DVL=diag([Simulation.Output.Kalman_mtx.R.r_vx,Simulation.Output.Kalman_mtx.R.r_vy,Simulation.Output.Kalman_mtx.R.r_vz]);
            R_Inclino = diag([Simulation.Output.Kalman_mtx.R.r_aroll,Simulation.Output.Kalman_mtx.R.r_apitch]);
            R_Heading = Simulation.Output.Kalman_mtx.R.r_yaw;
            Simulation.Output.Kalman_mtx.VB_adaptiveQR.U_hat(:,:,1) = coeff_U*Simulation.parameter_VB_adaptiveQR.tau* blkdiag(R_GPS,R_depth,R_DVL,R_Inclino,R_Heading)*(Simulation.Output.Kalman_mtx.VB_adaptiveQR.u_hat(update_counter,1)-m-1);
            Simulation.Output.Kalman_mtx.VB_adaptiveQR.t_hat(1,1) = (n+tau+1); %%%Simulation.parameter_VB_adaptiveQR.t0;
        %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@     
        %% 
        
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% information_filter
%            Y = ( Simulation.Output.Kalman_mtx.landa_P*(Simulation.Output.Kalman_mtx.A* Simulation.Output.Kalman_mtx.P * Simulation.Output.Kalman_mtx.A' + Simulation.Output.Kalman_mtx.G* Simulation.Output.Kalman_mtx.QQc* Simulation.Output.Kalman_mtx.G'))^(-1);
%            L = Y * Simulation.Output.Kalman_mtx.A* Y^(-1); %% information propagation coefficent
%            y= (Simulation.Output.Kalman_mtx.P)^(-1)*Simulation.Output.ESKF.dX(1,:)';
%            y = L* y;
%            Simulation.Output.Kalman_mtx.Y = Y; %%% information matrix
%            Simulation.Output.Kalman_mtx.y = y; 
  
end




