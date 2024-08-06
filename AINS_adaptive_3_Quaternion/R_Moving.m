%Tunning of R(the measurement error variance) elements
%TC: Tunning Coefficient
function [ Simulation ] = R_Moving( Simulation )
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Constant Parameters of the Earth
    R=6378137;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    Simulation.Output.Kalman_mtx.R.r_Lat = Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_Lat/(R^2);  
    Simulation.Output.Kalman_mtx.R.r_lon = Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_lon/(R^2); 
                        
    Simulation.Output.Kalman_mtx.R.r_alt=Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_alt;
    
    Simulation.Output.Kalman_mtx.R.r_vx=Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vx;
    Simulation.Output.Kalman_mtx.R.r_vy=Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vy;
    Simulation.Output.Kalman_mtx.R.r_vz=Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vz;

    Simulation.Output.Kalman_mtx.R.r_aroll  = Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_aroll;
    Simulation.Output.Kalman_mtx.R.r_apitch = Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_apitch;
    
    Simulation.Output.Kalman_mtx.R.r_yaw   = Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_yaw*pi/180*pi/180;%(rad)^2
end