function [ Simulation ] = Run_EKF( )

%       load('Real_Measuremet_test1_22th_2', 'Real_Measurement')
%         load('Real_Measurement_testReal_LSTS', 'Real_Measurement')
%         load('Parameter_stationary_testReal_LSTS', 'Simulation') 
%         load('Parameter_stationary_test1_22th', 'Simulation')


load('Real_Measurement_IMU_testReal_LSTS_113338_quad_ms_18Agust', 'Real_Measurement')
% load('Parameter_stationary_testReal_LSTS_112509_quad_rpm_rev','Simulation')
load('Parameter_stationary_testReal_LSTS_112509_quad_rpm_rev','Simulation')

        
        Simulation.Input.Measurements.IMU       = Real_Measurement.IMU;
        Simulation.Input.Measurements.Heading   = Real_Measurement.Heading;
        Simulation.Input.Measurements.Ref_Pos   = Real_Measurement.Ref_Pos;
        Simulation.Input.Measurements.DVL       = Real_Measurement.DVL;
        Simulation.Input.Measurements.Depth     = Real_Measurement.Depth;
        Simulation.Input.Measurements.GPS       = Real_Measurement.GPS;
               
        [Simulation]=IINS_2(Simulation);
end
