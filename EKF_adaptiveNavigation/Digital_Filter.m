%First Order Digital Filter
function [ Simulation ] = Digital_Filter( Simulation , S , dt , fc , I , SigType , ave_sample )


    if strcmp(SigType,'RollPitch')
        Simulation.Output.EKF.FilteredSignal.filtered_RP(I-ave_sample + 1,1:3)   = ...
        Simulation.Output.EKF.FilteredSignal.filtered_RP(I-ave_sample ,1:3) + 2 * pi * fc * dt * ...
        (S - Simulation.Output.EKF.FilteredSignal.filtered_RP(I-ave_sample,1:3));
    end
    if strcmp(SigType,'Accel')
        Simulation.Output.EKF.FilteredSignal.filtered_Accel(I-ave_sample + 1,1:3)   = ...
        Simulation.Output.EKF.FilteredSignal.filtered_Accel(I-ave_sample,1:3) + 2 * pi * fc * dt * ...
        (S - Simulation.Output.EKF.FilteredSignal.filtered_Accel(I-ave_sample,1:3));        
    end
    if strcmp(SigType,'Gyro')
        Simulation.Output.EKF.FilteredSignal.filtered_Gyro(I-ave_sample + 1,1:3)   = ...
        Simulation.Output.EKF.FilteredSignal.filtered_Gyro(I-ave_sample,1:3) + 2 * pi * fc * dt * ...
        (S - Simulation.Output.EKF.FilteredSignal.filtered_Gyro(I-ave_sample,1:3));        
    end  
end