%Strapdown inertial navigation system function (Related to the indirect approach, ESKF)
function [Simulation]=SINS(Simulation , I , dt ,func_f, fc_f_RP , fc_f_accel , fc_f_gyro , filtered_accel , filtered_gyro ,ave_sample,muu,coeff_final,include_MA)
                            
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    %Constant Parameters of the Earth
    R    =6378137;
    Omega=7.292115e-05;%Earth's rate
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    %stepsize
%     dt=1/fs;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if include_MA==1 && I>=ave_sample+4
                y=Simulation.Input.Measurements.IMU(I-3:I,2:4);
                bias=Simulation.Output.INS.X_INS(I-ave_sample-3:I-ave_sample,10:15);
                Simulation.Input.Measurements.IMU(I,2:4)=moving_average_real(y,bias,ave_sample,I,coeff_final,3);
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Recieving of  accel and gyro signals     
            fb(1)  = Simulation.Input.Measurements.IMU(I,2);
            fb(2)  = Simulation.Input.Measurements.IMU(I,3);
            fb(3)  = Simulation.Input.Measurements.IMU(I,4);
            
            fb_(1)  = Simulation.Input.Measurements.IMU(I-1,2);
            fb_(2)  = Simulation.Input.Measurements.IMU(I-1,3);
            fb_(3)  = Simulation.Input.Measurements.IMU(I-1,4);
            

            Wib_b(1)  = Simulation.Input.Measurements.IMU(I,5);   
            Wib_b(2)  = Simulation.Input.Measurements.IMU(I,6);
            Wib_b(3)  = Simulation.Input.Measurements.IMU(I,7);
            
            Wib_b_(1)  = Simulation.Input.Measurements.IMU(I-1,5);   
            Wib_b_(2)  = Simulation.Input.Measurements.IMU(I-1,6);
            Wib_b_(3)  = Simulation.Input.Measurements.IMU(I-1,7);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            [gl,g0] = Gravity(Simulation.Output.INS.X_INS(I-ave_sample,1:3));

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            [ Simulation ] = Digital_Filter( Simulation , fb    , dt , fc_f_RP    , I , 'RollPitch', ave_sample );
            [ Simulation ] = Digital_Filter( Simulation , fb    , dt , fc_f_accel , I , 'Accel', ave_sample);
            [ Simulation ] = Digital_Filter( Simulation , Wib_b , dt , fc_f_gyro  , I , 'Gyro', ave_sample);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            filtered_RP    = Simulation.Output.INS.FilteredSignal.filtered_RP(I- ave_sample + 1,1:3);
            filtered_Accel = Simulation.Output.INS.FilteredSignal.filtered_Accel(I- ave_sample + 1,1:3);
            filtered_Gyro  = Simulation.Output.INS.FilteredSignal.filtered_Gyro(I- ave_sample + 1,1:3);
            filtered_Accel_ = Simulation.Output.INS.FilteredSignal.filtered_Accel(I - ave_sample ,1:3);
            filtered_Gyro_  = Simulation.Output.INS.FilteredSignal.filtered_Gyro(I- ave_sample ,1:3);  
            Simulation.Output.INS.norm.Accel_norm(I- ave_sample + 1) = abs(norm(filtered_RP) - norm(gl));%fb
            Simulation.Output.INS.norm.Gyro_norm (I- ave_sample + 1) = norm(filtered_Gyro);%Wib_b               
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
             X_ = Simulation.Output.INS.X_INS(I - ave_sample,1:9);
             C_ = Simulation.Output.INS.Cbn(:,:,I - ave_sample);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if     ~filtered_accel && ~filtered_gyro
                    [fb,fb_,Wib_b,Wib_b_]=correction_Bias(Simulation,fb,fb_,Wib_b,Wib_b_,I,ave_sample);
                elseif filtered_accel && ~filtered_gyro
                    [fb,fb_,Wib_b,Wib_b_]=correction_Bias(Simulation,filtered_Accel,filtered_Accel_,Wib_b,Wib_b_,I,ave_sample);
                elseif ~filtered_accel && filtered_gyro
                    [fb,fb_,Wib_b,Wib_b_]=correction_Bias(Simulation,fb,fb_,filtered_Gyro,filtered_Gyro_);
                elseif filtered_accel && filtered_gyro
                    [fb,fb_,Wib_b,Wib_b_]=correction_Bias(Simulation,filtered_Accel,filtered_Accel_,filtered_Gyro,filtered_Gyro_,I,ave_sample);   
                end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % Rectangular Integration
        [X_dot , fnn , Wnb_b] = feval(func_f , X_ , C_ , fb_ , Wib_b_ , gl);
        delta_X = X_dot * dt;
        X = X_ + delta_X';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         if X(9) > pi
            X(9) = X(9) - 2*pi;
        elseif X(9) < -pi
            X(9) = X(9) + 2*pi;
        else
            %No Operation
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Limitation
        if X(3) > 300 
            X(3) = 300;
        end
        if X(3) < -5
            X(3) = -5;
        end
        if X(4) > 20
            X(4) = 20;
        end
        if X(4) < -20
            X(4) = -20;
        end    
        if X(5) > 20
            X(5) = 20;
        end
        if X(5) < -20
            X(5) = -20;
        end    
        if X(6) > 20
            X(6) = 20;
        end
        if X(6) < -20
            X(6) = -20;
        end    
        if X(7) > 30 * pi/180 
            X(7) = 30 * pi/180;
        end
        if X(7) < -30 * pi/180 
            X(7) = - 30 * pi/180;
        end 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            INS_Euler_I = [X(7),X(8),X(9)];
            Simulation.Output.INS.Cbn(:,:,I - ave_sample + 1) = InCBN(INS_Euler_I);%corrected transformation matrix             
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.INS.X_INS(I - ave_sample + 1,:) = [ X , Simulation.Output.INS.X_INS(I - ave_sample,10:15)];
            Simulation.Output.INS.fnn(I - ave_sample ,:)   = fnn;
            Simulation.Output.INS.fn(I - ave_sample + 1,:)    = X_dot(4:6,:)';
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %      if Simulation.Output.Alignment.norm.Accel_norm(I) <  mu
        %     if include_accelrollpitch
        %         [ Simulation ] = Alignment2( Simulation , filtered_RP , gl(3) , I , muu , ave_sample );%fb
        %         [ Simulation ] = Alignment3( Simulation , filtered_RP , gl(3) , I , muu , ave_sample );%fb               
                [ Simulation ] = Alignment4( Simulation , filtered_RP , gl(3) , I , muu , ave_sample );%fb   
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             %% Modeling bias with Random Walk
%             bias_ax = Simulation.Output.INS.X_INS(I - ave_sample,10);
%             bias_ay = Simulation.Output.INS.X_INS(I - ave_sample,11);
%             bias_az = Simulation.Output.INS.X_INS(I - ave_sample,12);
%             bias_gx = Simulation.Output.INS.X_INS(I - ave_sample,13);
%             bias_gy = Simulation.Output.INS.X_INS(I - ave_sample,14);
%             bias_gz = Simulation.Output.INS.X_INS(I - ave_sample,15);
            
% % %             %% Modeling bias with first order Gauss markov
%               bias_ax = (1-dt/tau(1))*Simulation.Output.INS.X_INS(I-1,10);
% %                 bias_ax = exp(-dt/tau(1))*Simulation.Output.INS.X_INS(I-1,10);    
%              bias_ay = (1-dt/tau(2))*Simulation.Output.INS.X_INS(I-1,11);
% %                 bias_ay = exp(-dt/tau(2))*Simulation.Output.INS.X_INS(I-1,11);
%              bias_az = (1-dt/tau(3))*Simulation.Output.INS.X_INS(I-1,12);
% %                bias_az = exp(-dt/tau(3))*Simulation.Output.INS.X_INS(I-1,12);
%               bias_gx = (1-dt/tau(4))*Simulation.Output.INS.X_INS(I-1,13);
% %                bias_gx = exp(-dt/tau(4))*Simulation.Output.INS.X_INS(I-1,13);
%              bias_gy = (1-dt/tau(5))*Simulation.Output.INS.X_INS(I-1,14);
% %                 bias_gy = exp(-dt/tau(5))*Simulation.Output.INS.X_INS(I-1,14);
%              bias_gz = (1-dt/tau(6))*Simulation.Output.INS.X_INS(I-1,15);
% %                 bias_gz = exp(-dt/tau(6))*Simulation.Output.INS.X_INS(I-1,15);
    %$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    %$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
end