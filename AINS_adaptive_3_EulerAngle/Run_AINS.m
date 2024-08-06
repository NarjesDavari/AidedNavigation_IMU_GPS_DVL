% function [ Simulation ] = Run_AINS(Simulation,Real_Measurement,s_final,include_MA )
function [ Simulation ] = Run_AINS( )

%       load('Real_Measurement_simulateddata_LSTS', 'Real_Measurement')
%         load('Parameter_stationary_simulatedData', 'Simulation') 
%         load('Real_Measurement_testReal_LSTS', 'Real_Measurement')
%         load('Parameter_stationary_3', 'Simulation') 
        load('Real_Measurement_testReal_LSTS_5Jun', 'Real_Measurement')
        load('Parameter_stationary_5Jun_1Sam', 'Simulation') 
        

        
        Simulation.Input.Measurements.IMU       = Real_Measurement.IMU;
        Simulation.Input.Measurements.Heading   = Real_Measurement.Heading;
        Simulation.Input.Measurements.Ref_Pos   = Real_Measurement.Ref_Pos;
        Simulation.Input.Measurements.DVL       = Real_Measurement.DVL;
        Simulation.Input.Measurements.Depth     = Real_Measurement.Depth;
        Simulation.Input.Measurements.GPS       = Real_Measurement.GPS;
               
        coeff_MA = Simulation.Parameters_denoising.coeff_final;
        include_MA = Simulation.Parameters_denoising.include_MA; 
        Simulation = IINS_2(Simulation,coeff_MA,include_MA);
end
