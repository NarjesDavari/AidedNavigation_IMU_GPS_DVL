function [ Simulation ] = Run_AINS_1( )


%          load('Real_Measurement_testReal_LSTS_5Jun', 'Real_Measurement')
%         load('Parameter_stationary_5Jun_1Sam', 'Simulation') 

%         load('Parameter_stationary_simulatedData', 'Simulation')
%         load('Real_Measurement_simulateddata_LSTS_3', 'Real_Measurement')

load('Real_Measurement_testReal_LSTS_112509_quad_rpm_rev', 'Real_Measurement')
% load('Parameter_stationary_adaptive_testReal_LSTS_112509_quad_rpm_rev','Simulation')
load('Parameter_stationary','Simulation')

% load('Parameter_stationary_testReal_LSTS_145927_line_ms','Simulation')
% load('Real_Measurement_testReal_LSTS_145927_line_ms','Real_Measurement')
        
        Simulation.Input.Measurements.IMU       = Real_Measurement.IMU;
        Simulation.Input.Measurements.Compass   = Real_Measurement.Compass;
        Simulation.Input.Measurements.Ref_Pos   = Real_Measurement.Ref_Pos;
        Simulation.Input.Measurements.DVL       = Real_Measurement.DVL;
        Simulation.Input.Measurements.Depth     = Real_Measurement.Depth;
        Simulation.Input.Measurements.GPS       = Real_Measurement.GPS;
               
        coeff_MA = Simulation.Parameters_denoising.coeff_final;
        include_MA = Simulation.Parameters_denoising.include_MA; 
        Simulation = IINS_2(Simulation,coeff_MA,include_MA);
end
