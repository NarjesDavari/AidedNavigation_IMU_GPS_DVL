
function [ Simulation ] = Run_AINS_3( )

%       load('Real_Measurement_simulateddata_LSTS', 'Real_Measurement')
%         load('Parameter_stationary_simulatedData', 'Simulation') 
%         load('Real_Measurement_testReal_LSTS', 'Real_Measurement')
%         load('Parameter_stationary_3', 'Simulation') 

%         load('Real_Measurement_testReal_LSTS_5Jun', 'Real_Measurement')
%         load('Parameter_stationary_5Jun_1Sam_outlierDete', 'Simulation') 
     
load('Real_Measurement_LSTS_Survey_113801_NP3_A44_cutted', 'Real_Measurement')
load('Parameter_stationary_LSTS_Survey_113801_NP3_A44_1','Simulation')

% load('Real_Measurement_testReal_LSTS_112509_quad_rpm_rev', 'Real_Measurement')
% load('Parameter_stationary_testReal_LSTS_112509_quad_rpm_rev','Simulation')

%         load('Real_Measurement_testReal_LSTS_145927_line_ms','Real_Measurement')
%         load('Parameter_stationary_testReal_LSTS_145927_line_ms','Simulation')

% load('Real_Measurement_testReal_LSTS_AHRS_112509_quad_rpm_rev', 'Real_Measurement')
% load('Parameter_stationary_testReal_LSTS_112509_quad_rpm_rev2','Simulation')
        

        
        Simulation.Input.Measurements.IMU       = Real_Measurement.IMU;
        Simulation.Input.Measurements.Heading   = Real_Measurement.Heading;
        Simulation.Input.Measurements.Ref_Pos   = Real_Measurement.Ref_Pos;
        Simulation.Input.Measurements.DVL       = Real_Measurement.DVL;
        Simulation.Input.Measurements.DVL_new       = Real_Measurement.DVL;
        Simulation.Input.Measurements.Depth     = Real_Measurement.Depth;
        Simulation.Input.Measurements.GPS       = Real_Measurement.GPS;
               
        coeff_MA = Simulation.Parameters_denoising.coeff_final;
        include_MA = Simulation.Parameters_denoising.include_MA; 
        Simulation = IINS_2(Simulation,coeff_MA,include_MA);
end
