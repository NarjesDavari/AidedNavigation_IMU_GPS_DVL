%Tunning of Qc(power spectral density) elements

function [ Simulation ] = Qc_Moving( Simulation )
  
    Simulation.Output.Kalman_mtx.Qc.q_ax = Simulation.Parameters_IMUNoisePSD_Nav.PSD_ax;  
    Simulation.Output.Kalman_mtx.Qc.q_ay = Simulation.Parameters_IMUNoisePSD_Nav.PSD_ay; 
    Simulation.Output.Kalman_mtx.Qc.q_az = Simulation.Parameters_IMUNoisePSD_Nav.PSD_az; 
    
    Simulation.Output.Kalman_mtx.Qc.q_wx = Simulation.Parameters_IMUNoisePSD_Nav.PSD_wx;
    Simulation.Output.Kalman_mtx.Qc.q_wy = Simulation.Parameters_IMUNoisePSD_Nav.PSD_wy;
    Simulation.Output.Kalman_mtx.Qc.q_wz = Simulation.Parameters_IMUNoisePSD_Nav.PSD_wz;    
    
    Simulation.Output.Kalman_mtx.Qc.q_Bax = Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bax;  
    Simulation.Output.Kalman_mtx.Qc.q_Bay = Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bay; 
    Simulation.Output.Kalman_mtx.Qc.q_Baz = Simulation.Parameters_IMUNoisePSD_Nav.PSD_Baz; 
    
    Simulation.Output.Kalman_mtx.Qc.q_Bwx = Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bwx;
    Simulation.Output.Kalman_mtx.Qc.q_Bwy = Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bwy;
    Simulation.Output.Kalman_mtx.Qc.q_Bwz = Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bwz;    
    
end