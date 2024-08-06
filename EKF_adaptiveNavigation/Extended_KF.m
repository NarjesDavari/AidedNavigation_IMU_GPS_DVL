function [ Simulation ] = Extended_KF( Simulation , I  , fc_f_RP , fc_f_accel , fc_f_gyro , dt , ...
                                       filtered_accel , filtered_gyro , func_f , Selection_Param , C_DVL_IMU,ave_sample )

    mu = Selection_Param{11};
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Recieving of  accel and gyro signals
    fb_(1)=Simulation.Input.Measurements.IMU(I-1,2);
    fb_(2)=Simulation.Input.Measurements.IMU(I-1,3);
    fb_(3)=Simulation.Input.Measurements.IMU(I-1,4);
    
    fb(1)=Simulation.Input.Measurements.IMU(I,2);
    fb(2)=Simulation.Input.Measurements.IMU(I,3);
    fb(3)=Simulation.Input.Measurements.IMU(I,4);

    Wib_b_(1)=Simulation.Input.Measurements.IMU(I-1,5);    
    Wib_b_(2)=Simulation.Input.Measurements.IMU(I-1,6);    
    Wib_b_(3)=Simulation.Input.Measurements.IMU(I-1,7);
    
    Wib_b(1)=Simulation.Input.Measurements.IMU(I,5);    
    Wib_b(2)=Simulation.Input.Measurements.IMU(I,6);    
    Wib_b(3)=Simulation.Input.Measurements.IMU(I,7);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Posn_ = Simulation.Output.EKF.X(I-1,1:3);
    gl = Gravity( Posn_  );    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [ Simulation ] = Digital_Filter( Simulation , fb    , dt , fc_f_RP    , I , 'RollPitch' , ave_sample);
    [ Simulation ] = Digital_Filter( Simulation , fb    , dt , fc_f_accel , I , 'Accel'     , ave_sample);
    [ Simulation ] = Digital_Filter( Simulation , Wib_b , dt , fc_f_gyro  , I , 'Gyro'      , ave_sample);  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    filtered_RP    = Simulation.Output.EKF.FilteredSignal.filtered_RP(I,1:3);
    filtered_Accel = Simulation.Output.EKF.FilteredSignal.filtered_Accel(I,1:3);
    filtered_Gyro  = Simulation.Output.EKF.FilteredSignal.filtered_Gyro(I,1:3);
    filtered_Accel_ = Simulation.Output.EKF.FilteredSignal.filtered_Accel(I-1,1:3);
    filtered_Gyro_  = Simulation.Output.EKF.FilteredSignal.filtered_Gyro(I-1,1:3);    
    Simulation.Output.Alignment.norm.Accel_norm(I) = abs(norm(filtered_RP) - norm(gl));%fb
    Simulation.Output.Alignment.norm.Gyro_norm (I) = norm(filtered_Gyro);%Wib_b            
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if     ~filtered_accel && ~filtered_gyro
        [fb,fb_,Wib_b,Wib_b_]=correction_Bias(Simulation,fb,fb_,Wib_b,Wib_b_,I,ave_sample);
    elseif filtered_accel && ~filtered_gyro
        [fb,fb_,Wib_b,Wib_b_]=correction_Bias(Simulation,filtered_Accel,filtered_Accel_,Wib_b,Wib_b_,I,ave_sample);
    elseif ~filtered_accel && filtered_gyro
        [fb,fb_,Wib_b,Wib_b_]=correction_Bias(Simulation,fb,fb_,filtered_Gyro,filtered_Gyro_);
    elseif filtered_accel && filtered_gyro
        [fb,fb_,Wib_b,Wib_b_]=correction_Bias(Simulation,filtered_Accel,filtered_Accel_,filtered_Gyro,filtered_Gyro_,I,ave_sample);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    X_ = Simulation.Output.EKF.X(I-1,:);
    P_ = Simulation.Output.Kalman_mtx.P ;
    A_ = Simulation.Output.Kalman_mtx.A;
    Q_ = Simulation.Output.Kalman_mtx.Q;
    G_ = Simulation.Output.Kalman_mtx.G;
    C_ = Simulation.Output.EKF.Cbn(:,:,I-1);
    if     ~filtered_accel && ~filtered_gyro
        [X , P , fnn , Wnb_b ] = ekf_predict1(X_ , P_ , func_f , A_ , Q_ , C_ , fb , fb_ , Wib_b , Wib_b_ , gl , dt);
    elseif filtered_accel && ~filtered_gyro
        [X , P , fnn , Wnb_b ] = ekf_predict1(X_ , P_ , func_f , A_ , Q_ , C_ , filtered_Accel , filtered_Accel_ , Wib_b , Wib_b_ , gl , dt);
    elseif ~filtered_accel && filtered_gyro
        [X , P , fnn , Wnb_b ] = ekf_predict1(X_ , P_ , func_f , A_ , Q_ , C_ , fb , fb_ , filtered_Gyro , filtered_Gyro_ , gl , dt);
    elseif filtered_accel && filtered_gyro
        [X , P , fnn , Wnb_b ] = ekf_predict1(X_ , P_ , func_f , A_ , Q_ , C_ , filtered_Accel , filtered_Accel_ , filtered_Gyro , filtered_Gyro_ , gl , dt);
    end
    INS_Euler_I                      = [X(7),X(8),X(9)] ;
    Simulation.Output.EKF.Cbn(:,:,I) = InCBN(INS_Euler_I);%Predicted transformation matrix
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [ Simulation ] = EKF_Correction_Param(Simulation,Selection_Param,I,C_DVL_IMU ,ave_sample,dt);    
    H = Simulation.Output.Kalman_mtx.H;
    R = Simulation.Output.Kalman_mtx.R.Rmatrx;
    Y = Simulation.Output.Kalman_mtx.z;
    if (~isempty (Simulation.Output.Kalman_mtx.H))
       
        
        if Simulation.select_adaptive.VB
            alpha=Simulation.Output.Kalman_mtx.alpha_VB;
            beta=Simulation.Output.Kalman_mtx.beta_VB;
            rho=1-exp(-Simulation.Output.Kalman_mtx.rho_VB);
            alpha_ = rho'.*alpha;
            beta_ = rho'.*beta;
%            [Simulation,X,P,alpha,beta]     =     EKF_update_VB_IW(Simulation,X,P,Y,H,alpha_,beta_,I); 
           [Simulation,X,P,alpha,beta]    =   EKF_update_VB(Simulation,X_,P_,Y,H,alpha_,beta_,I);
           Simulation.Output.Kalman_mtx.alpha_VB=alpha;
           Simulation.Output.Kalman_mtx.beta_VB=beta;
           
        elseif Simulation.select_adaptive.VB_adaptiveQR 
            [Simulation,X,P]= VB_QR_Estimat(Simulation,X_,P_, Y ,H , I);
                       
        else
        [X , P]      =       ekf_update1(X , P , Y , H , R ,I);
%         [Simulation,X,P] = EKF_update_2(Simulation, X , Y , H , R,I);
        Simulation.Output.Kalman_mtx.P_plus = P;
        Simulation.Output.Kalman_mtx.X_plus = X;
        end
        INS_Euler_I                      = [X(7),X(8),X(9)] ;
        Simulation.Output.EKF.Cbn(:,:,I) = InCBN(INS_Euler_I);%Correted transformation matrix        
         Simulation.Output.Kalman_mtx.update_counter = Simulation.Output.Kalman_mtx.update_counter+1;
    end
    
    Simulation.Output.Kalman_mtx.P = P;
%     Simulation.Output.EKF.X(I,:) = [ X , X_dot(4:6,:)' , Wnb_b];
Simulation.Output.EKF.X(I,:) = X ;
    Simulation.Output.EKF.fnn(I,:)=fnn;
    Simulation.Output.Kalman_mtx.P_diag(I,:)=[P(1,1) P(2,2) P(3,3) P(4,4) P(5,5) P(6,6) P(7,7) P(8,8) P(9,9) P(10,10) P(11,11) P(12,12) P(13,13) P(14,14) P(15,15)];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if Simulation.Output.Alignment.norm.Accel_norm(I) <  mu
        [ Simulation ] = Alignment( Simulation ,  I , filtered_RP , gl(3) );%fb
        Simulation.Output.Alignment.time(Simulation.Output.Alignment.RP_counter,1)  = Simulation.Input.Measurements.IMU(I,1);
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    x = Simulation.Output.EKF.X(I,1:9); 
    C = Simulation.Output.EKF.Cbn(:,:,I);
    [Simulation]=AQ_calcul(x , fb , Wib_b , C , Simulation , 1/dt , I);     
end

