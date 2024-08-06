%Initialization of the orientation, transformation matrix, vvelocity,
%position, acceleration, and so on.
function [ Simulation ] = Initialization( Simulation , fs ,ave_sample)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            global Updt_Cntr;
            global Cbn_det;
            Updt_Cntr = 0;  
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    
            Dlength = length(Simulation.Input.Measurements.IMU);     
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Input.Measurements.GPS_Counter   = 1; %GPS Counter
            Simulation.Input.Measurements.DVL_Counter   = 1; %DVL Counter%Path4:1658
            Simulation.Input.Measurements.Depth_Counter = 1; %Depth Counter
            Simulation.Input.Measurements.hdng_Counter  = 1; %Heading Counter
            Simulation.Input.Measurements.incln_Counter = 1; %Inclinometer Counter
            Simulation.Input.Measurements.accelrollpitch_Counter=1;
            
            Simulation.Input.Measurements.GPS_Counter_fusion=0;
            Simulation.Input.Measurements.Depth_Counter_fusion=0;
            Simulation.Input.Measurements.DVL_Counter_fusion =0;
            Simulation.Input.Measurements.hdng_Counter_fusion=0;
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %states                
            X1                            = (zeros(1,15))';%column vector
            Simulation.Output.EKF.X      = zeros(Dlength-ave_sample +1 ,size(X1,1));
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Cbn_det   = zeros(Dlength-ave_sample,1);  
            
             %Initial acceleration and angular velocity
            W1ib_b(1)=Simulation.Input.Measurements.IMU(1,5);    
            W1ib_b(2)=Simulation.Input.Measurements.IMU(1,6);    
            W1ib_b(3)=Simulation.Input.Measurements.IMU(1,7);
            
            f1b(1)=Simulation.Input.Measurements.IMU(1,2);
            f1b(2)=Simulation.Input.Measurements.IMU(1,3);
            f1b(3)=Simulation.Input.Measurements.IMU(1,4);     
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             %%Alignment:computation of initial transformation matrix
            [ Simulation , gl , ave_fb , ave_W ] = Coarse_Alignment( Simulation,ave_sample  );
            
            %%Alignment:computation of initial transformation matrix
            Euler1=Simulation.Output.EKF.X(1,7:9);
            Simulation.Input.InitialParam.InitialEuler  = Euler1;
            Cbn                                     = InCBN(Euler1);
            Simulation.Output.EKF.Cbn                   = zeros(3,3,Dlength-ave_sample +1 );          
%             Simulation.Output.EKF.Cbn        = zeros(3,3,Dlength-ave_sample);
            Simulation.Output.EKF.Cbn(:,:,1) = Cbn;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Initial velocity in body frame
            Simulation.Input.InitialParam.InitialVelocity            =   Simulation.Output.EKF.X(1,4:6);   
            V1n=Simulation.Output.EKF.X(1,4:6); 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Initial Position in radian
            Pos1rad=Simulation.Output.EKF.X(1,1:3);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           
                   
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.EKF.FilteredSignal.filtered_RP      = zeros(Dlength-ave_sample +1,3);
            Simulation.Output.EKF.FilteredSignal.filtered_RP(1,:) = ave_fb;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.EKF.FilteredSignal.filtered_Accel      = zeros(Dlength-ave_sample +1,3);
            Simulation.Output.EKF.FilteredSignal.filtered_Accel(1,:) = f1b;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.EKF.FilteredSignal.filtered_Gyro      = zeros(Dlength-ave_sample +1,3);
            Simulation.Output.EKF.FilteredSignal.filtered_Gyro(1,:) = ave_W;            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             gl = Gravity( Pos1rad , gg );
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
            Simulation.Output.Alignment.norm.Accel_norm = zeros(Dlength-ave_sample +1,1);
            Simulation.Output.Alignment.norm.Gyro_norm  = zeros(Dlength-ave_sample +1,1);
            Simulation.Output.Alignment.norm.Accel_norm(1) = abs(norm(ave_fb) - norm(gl));
            Simulation.Output.Alignment.norm.Gyro_norm (1) = norm(ave_W);       
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
            %computation of the angular rate of the navigation frame with
            %respect to the inertial frame              
            Win_n = Win_n_calcul( Simulation.Output.EKF.X(1,1:6)  );            
            %computation of the body rate with respect to the navigation frame.
            Wnb_b=(W1ib_b'-Simulation.Output.EKF.Cbn(:,:,1)'*Win_n)';             
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           
            Simulation.Output.Alignment.phi   = zeros(Dlength-ave_sample +1,1);
            Simulation.Output.Alignment.theta = zeros(Dlength-ave_sample +1,1);
            Simulation.Output.Alignment.time  = zeros(Dlength-ave_sample +1,1);
            Simulation.Output.Alignment.RP_counter = 1;
            
            Simulation.Output.Alignment.theta(1,1) = Simulation.Output.EKF.X(1,8);
            Simulation.Output.Alignment.phi(1,1)   = Simulation.Output.EKF.X(1,7);
            Simulation.Output.Alignment.ave_theta(1,1)   = Simulation.Output.EKF.X(1,8);
            Simulation.Output.Alignment.ave_phi(1,1)     = Simulation.Output.EKF.X(1,7);      
            Simulation.Output.Alignment.time(1,1)  = Simulation.Input.Measurements.IMU(1,1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
            W_Coriolis = Coriolis_correction( Simulation.Output.EKF.X(1,1:6));
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.EKF.fnn  =  zeros(Dlength-ave_sample ,3);
            f1nn       =   (Cbn*ave_fb')';%%convert initial Accelerometer frome  body to navigation
            f1n        =       f1nn-cross(W_Coriolis,V1n)+gl;
            Simulation.Output.EKF.fn(1,:) =  f1n;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %position,velocity,euler angles and accel computed by SDINS in navigation
            %frame
%             X1=[Pos1rad,V1n,Euler1,f1n,Wnb_b]';%n_frame
            X1=Simulation.Output.EKF.X(1,:);%n_frame
            
%             Simulation.Output.EKF.X(1,:) = X1;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Input.Measurements.GPS_Counter   = 1; %GPS Counter
            Simulation.Input.Measurements.DVL_Counter   = 1; %DVL Counter
            Simulation.Input.Measurements.Depth_Counter = 1; %Depth Counte
            Simulation.Input.Measurements.incln_Counter = 1; %Inclinometer Counter
            Simulation.Input.Measurements.hdng_Counter  = 1; %Heading Counter
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.Kalman_mtx.Q_diag=zeros(Dlength-ave_sample,9);
            %corrected position,velocity and accel(in navigation frame)
            [Simulation]=AQ_calcul(X1, ave_fb , ave_W ,Cbn ,Simulation,fs,1);
                        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.Kalman_mtx.X_plus = X1;
            Simulation.Output.Kalman_mtx.P = zeros(15,15);
%             b1=Simulation.Init_Value.Init_Bias_P; 
b1= 1e-4;
            Simulation.Output.Kalman_mtx.P(1,1) = 1e-6*b1;
            Simulation.Output.Kalman_mtx.P(2,2) = 1e-6*b1;
            Simulation.Output.Kalman_mtx.P(3,3) = 1e-6*b1;
            Simulation.Output.Kalman_mtx.P(4,4) = 1e-6*b1;
            Simulation.Output.Kalman_mtx.P(5,5) = 1e-6*b1;
            Simulation.Output.Kalman_mtx.P(6,6) = 1e-6*b1;
            Simulation.Output.Kalman_mtx.P(7,7) =  1e-6*b1;
            Simulation.Output.Kalman_mtx.P(8,8) = 1e-6*b1;
            Simulation.Output.Kalman_mtx.P(9,9) =  1e-6*b1;
            Simulation.Output.Kalman_mtx.P(10,10) = 1e-6*b1;
            Simulation.Output.Kalman_mtx.P(11,11) = 1e-6*b1;
            Simulation.Output.Kalman_mtx.P(12,12) =  1e-6*b1;
            Simulation.Output.Kalman_mtx.P(13,13) =  1e-6*b1;
            Simulation.Output.Kalman_mtx.P(14,14) =  1e-6*b1;
            Simulation.Output.Kalman_mtx.P(15,15) =  1e-6*b1;
%             Simulation.Output.Kalman_mtx.P0_Bias=[Simulation.Output.Kalman_mtx.P(10,10) Simulation.Output.Kalman_mtx.P(11,11) Simulation.Output.Kalman_mtx.P(12,12)...
%                Simulation.Output.Kalman_mtx.P(13,13) Simulation.Output.Kalman_mtx.P(14,14)  Simulation.Output.Kalman_mtx.P(15,15)];
            
            Simulation.Output.Kalman_mtx.P_diag=zeros(Dlength-ave_sample,size(X1,2));
            Simulation.Output.Kalman_mtx.P_diag(1,:)=[Simulation.Output.Kalman_mtx.P(1,1) Simulation.Output.Kalman_mtx.P(2,2) Simulation.Output.Kalman_mtx.P(3,3)...
                                                      Simulation.Output.Kalman_mtx.P(4,4) Simulation.Output.Kalman_mtx.P(5,5) Simulation.Output.Kalman_mtx.P(6,6)...
                                                      Simulation.Output.Kalman_mtx.P(7,7) Simulation.Output.Kalman_mtx.P(8,8) Simulation.Output.Kalman_mtx.P(9,9)...
                                                      Simulation.Output.Kalman_mtx.P(10,10) Simulation.Output.Kalman_mtx.P(11,11) Simulation.Output.Kalman_mtx.P(12,12)...
                                                      Simulation.Output.Kalman_mtx.P(13,13) Simulation.Output.Kalman_mtx.P(14,14)  Simulation.Output.Kalman_mtx.P(15,15)]';
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
            Simulation.Output.Kalman_mtx.P_plus = diag(Simulation.Output.Kalman_mtx.P_diag(1,:));
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Variational Bayesian
            Simulation.Output.Kalman_mtx.update_counter =1;
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
             
             Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(1,1)=Simulation.Output.Kalman_mtx.R.r_Lat;
             Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(1,2)=Simulation.Output.Kalman_mtx.R.r_lon;
             Simulation.Output.Kalman_mtx.R_adaptive.R_Depth(1,1)=Simulation.Output.Kalman_mtx.R.r_alt;
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(1,1)=Simulation.Output.Kalman_mtx.R.r_vx;
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(1,2)=Simulation.Output.Kalman_mtx.R.r_vy;
             Simulation.Output.Kalman_mtx.R_adaptive.R_DVL(1,3)=Simulation.Output.Kalman_mtx.R.r_vz;
             Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(1,1)=Simulation.Output.Kalman_mtx.R.r_aroll;
             Simulation.Output.Kalman_mtx.R_adaptive.R_accelrollpitch(1,2)=Simulation.Output.Kalman_mtx.R.r_apitch;
             Simulation.Output.Kalman_mtx.R_adaptive.R_Heading(1,1)=Simulation.Output.Kalman_mtx.R.r_yaw;
        %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             %%% parameters of VB_adaptive Q and R
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
            Simulation.Output.Kalman_mtx.VB_adaptiveQR.U_hat(:,:,1) = Simulation.parameter_VB_adaptiveQR.tau* blkdiag(R_GPS,R_depth,R_DVL,R_Inclino,R_Heading)*(Simulation.Output.Kalman_mtx.VB_adaptiveQR.u_hat(update_counter,1)-m-1);
            Simulation.Output.Kalman_mtx.VB_adaptiveQR.t_hat(1,1) = (n+tau+1); %%%Simulation.parameter_VB_adaptiveQR.t0;
        %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@        
      
        
end