%Tunning of R(the measurement error variance) elements
%TC: Tunning Coefficient
function [ Simulation ] = R_setting( Simulation )
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Constant Parameters of the Earth
    R=6378137;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    var_Lat             = Simulation.Parameters_AuxSnsrNoiseVar.var_Lat;
    TC_rLat             = Simulation.Parameters_AuxSnsrNoiseVar.TC_rLat;
    
    var_lon             = Simulation.Parameters_AuxSnsrNoiseVar.var_lon;
    TC_rlon             = Simulation.Parameters_AuxSnsrNoiseVar.TC_rlon;
    
    
    var_vx             = Simulation.Parameters_AuxSnsrNoiseVar.var_vx;
    var_vy             = Simulation.Parameters_AuxSnsrNoiseVar.var_vy;
    var_vz             = Simulation.Parameters_AuxSnsrNoiseVar.var_vz;
    TC_rvx             = Simulation.Parameters_AuxSnsrNoiseVar.TC_rvx;
    TC_rvy             = Simulation.Parameters_AuxSnsrNoiseVar.TC_rvy;
    TC_rvz             = Simulation.Parameters_AuxSnsrNoiseVar.TC_rvz;
    
    var_alt            = Simulation.Parameters_AuxSnsrNoiseVar.var_alt;
    TC_ralt            = Simulation.Parameters_AuxSnsrNoiseVar.TC_ralt;
    
    var_roll           = Simulation.Parameters_AuxSnsrNoiseVar.var_roll;
    var_pitch          = Simulation.Parameters_AuxSnsrNoiseVar.var_pitch;
    var_yaw            = Simulation.Parameters_AuxSnsrNoiseVar.var_yaw;
    TC_rphi            = Simulation.Parameters_AuxSnsrNoiseVar.TC_rphi;
    TC_rtheta          = Simulation.Parameters_AuxSnsrNoiseVar.TC_rtheta;
    TC_rpsi            = Simulation.Parameters_AuxSnsrNoiseVar.TC_rpsi;
    
    var_aroll          = Simulation.Parameters_AuxSnsrNoiseVar.var_aroll;
    var_apitch         = Simulation.Parameters_AuxSnsrNoiseVar.var_apitch;
    TC_arphi           = Simulation.Parameters_AuxSnsrNoiseVar.TC_arphi;
    TC_artheta         = Simulation.Parameters_AuxSnsrNoiseVar.TC_artheta;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    r_Lat=var_Lat*TC_rLat;    
    r_lon=var_lon*TC_rlon;   
    
    r_alt=var_alt*TC_ralt;
    
    r_vx=var_vx*TC_rvx;
    r_vy=var_vy*TC_rvy;
    r_vz=var_vz*TC_rvz; 
    
    r_roll=var_roll*TC_rphi;
    r_pitch=var_pitch*TC_rtheta;
    r_yaw=var_yaw*TC_rpsi;    
    
    r_aroll=var_aroll*TC_arphi;
    r_apitch=var_apitch*TC_artheta;    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    Simulation.Output.Kalman_mtx.R.r_Lat=r_Lat/(R^2);  
    Simulation.Output.Kalman_mtx.R.r_lon=r_lon/(R^2); 
                        
    Simulation.Output.Kalman_mtx.R.r_alt=r_alt;
    
    Simulation.Output.Kalman_mtx.R.r_vx=r_vx;
    Simulation.Output.Kalman_mtx.R.r_vy=r_vy;
    Simulation.Output.Kalman_mtx.R.r_vz=r_vz;
    
    Simulation.Output.Kalman_mtx.R.r_roll=r_roll;
    Simulation.Output.Kalman_mtx.R.r_pitch=r_pitch;
    Simulation.Output.Kalman_mtx.R.r_yaw=r_yaw;

    Simulation.Output.Kalman_mtx.R.r_aroll=r_aroll;
    Simulation.Output.Kalman_mtx.R.r_apitch=r_apitch;
end