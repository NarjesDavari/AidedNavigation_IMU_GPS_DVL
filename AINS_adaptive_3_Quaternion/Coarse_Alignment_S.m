function [ Simulation , gl ] = Coarse_Alignment_S( Simulation  )    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Dlength = length(Simulation.Input.Measurements.IMU);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %position,velocity,euler angles and accel computed by SDINS in navigation frame
    Simulation.Output.INS.X_INS      = zeros(Dlength,15);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Initial Position (Latitude, longitude, altitude)
    Simulation.Output.INS.X_INS(1,1) = Simulation.Init_Value.InitPosdeg(1) * pi/180;
    Simulation.Output.INS.X_INS(1,2) = Simulation.Init_Value.InitPosdeg(2) * pi/180;
    Simulation.Output.INS.X_INS(1,3) = Simulation.Init_Value.InitPosdeg(3) ;
          
    %Initial roll, pitch, heading
    Simulation.Output.INS.X_INS(1,7:9) = Simulation.Init_Value.InitialEuler;
    
    %Initial velocity
    Simulation.Output.INS.X_INS(1,4:6) = Simulation.Init_Value.InitialVelocity;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    gl = Gravity( [Simulation.Output.INS.X_INS(1,1), Simulation.Output.INS.X_INS(1,2),Simulation.Output.INS.X_INS(1,3)] );
  
    end

