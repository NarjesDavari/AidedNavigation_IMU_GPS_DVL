function [ Simulation ] = Alignment4( Simulation , fb , g , I , muu , ave_sample )
        global Accel_Count;
        global Attitude_Valid;
        global Roll_Sum;
        global Pitch_Sum;        
        global frp;
        Srp=100/frp;
        Accel_Count = Accel_Count + 1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Simulation.Output.Alignment.norm.Gyro_norm (I- ave_sample + 1)
        if Simulation.Output.INS.norm.Accel_norm(I - ave_sample + 1,1) <  muu
            Simulation.Output.Alignment.RP_counter = Simulation.Output.Alignment.RP_counter + 1;%Attitude_Valid
    
            Simulation.Output.Alignment.theta(I - ave_sample + 1,1)  = asin((fb(1))/g);
            
            Simulation.Output.Alignment.phi(I - ave_sample + 1,1)    = -asin((fb(2))/...
                                                                    (g*cos(Simulation.Output.Alignment.theta(I - ave_sample + 1))));
            Roll_Sum  = Roll_Sum  + Simulation.Output.Alignment.phi(I - ave_sample + 1,1);
            Pitch_Sum = Pitch_Sum + Simulation.Output.Alignment.theta(I - ave_sample + 1,1);
            Attitude_Valid = Attitude_Valid + 1;
        end
%         Simulation.Output.Alignment.time(Simulation.Output.Alignment.RP_counter,1) = Simulation.Input.Measurements.IMU(I,1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if Accel_Count>=Srp && Attitude_Valid~=0
%             Simulation.Output.Alignment.RP_counter2 = Simulation.Output.Alignment.RP_counter2 + 1;
%             J = Simulation.Output.Alignment.RP_counter2;
                Simulation.Output.Alignment.RP_active = 1;
                
                Simulation.Output.Alignment.ave_RP_counter = Simulation.Output.Alignment.ave_RP_counter + 1;
                
                Simulation.Output.Alignment.ave_phi(Simulation.Output.Alignment.ave_RP_counter,1)   = Roll_Sum/Attitude_Valid;
                
                Simulation.Output.Alignment.ave_theta(Simulation.Output.Alignment.ave_RP_counter,1) = Pitch_Sum/Attitude_Valid;
                
                Accel_Count = 0; Roll_Sum =0; Pitch_Sum=0;
                Attitude_Valid =0;
                
        end
end

