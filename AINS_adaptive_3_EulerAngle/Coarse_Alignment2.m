function [ Simulation , gl, ave_fb , ave_W ] = Coarse_Alignment2( Simulation , ave_sample )    

    %Five miutes:5*60sec=300sec *100sample=30000sample
%     ave_sample = 30000;
%     ave_sample = 5999;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Dlength = length(Simulation.Input.Measurements.IMU);
    dt=Simulation.Input.Measurements.IMU(2,1)-Simulation.Input.Measurements.IMU(1,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fb= zeros(ave_sample,3);
    W = zeros(ave_sample,3);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    CA.Roll  = zeros(ave_sample,1);
    CA.Pitch = zeros(ave_sample,1);
    CA.Yaw   = zeros(ave_sample,1);
    
    Simulation.Output.INS.X_INS      = zeros(Dlength-ave_sample+1 ,15);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      step_Heading=min(find(round(Simulation.Input.Measurements.Heading(:,1))==round(ave_sample*dt)+1));
      Simulation.Input.Measurements.hdng_Counter=step_Heading;
      step_GPS =min(find(round(Simulation.Input.Measurements.GPS(:,1))==round(ave_sample*dt)+1));
      Simulation.Input.Measurements.GPS_Counter=step_GPS;
      step_depth =min(find(round(Simulation.Input.Measurements.Depth(:,1))==round(ave_sample*dt)+1));
      Simulation.Input.Measurements.Depth_Counter=step_depth;
      step_DVL =min(find(round(Simulation.Input.Measurements.DVL(:,1))==round(ave_sample*dt)+1));
      Simulation.Input.Measurements.DVL_Counter=step_DVL;
      Lat= mean(Simulation.Input.Measurements.GPS(1:step_GPS,2)*(pi/180));
      Lon= mean(Simulation.Input.Measurements.GPS(1:step_GPS,3)*(pi/180));
      Z= mean(Simulation.Input.Measurements.Depth(1:step_depth,2));
      Yaw(:,1) = Simulation.Input.Measurements.Heading(1:step_Heading,2)*pi/180;
      for i=1:step_Heading
          if Yaw(i,1) > pi
              Yaw(i,1) = Yaw(i,1) - 2*pi;
          elseif Yaw(i,1)< -pi
              Yaw(i,1) =Yaw(i,1) + 2*pi;
          else
              Yaw(i,1) = Yaw(i,1);
          end
      end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [gl,g0] = Gravity( [Lat, Lon,Z] );
    fb = Simulation.Input.Measurements.IMU(1:ave_sample,2:4);
    W = Simulation.Input.Measurements.IMU(1:ave_sample,5:7);
    Pitch = asin((fb(:,1))./gl(3));
    Roll  = -asin((fb(:,2))./(gl(3).*cos(CA.Pitch)));    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Simulation.Output.INS.X_INS(1,1) = Lat;
    Simulation.Output.INS.X_INS(1,2) = Lon;
    Simulation.Output.INS.X_INS(1,3) = Z;
    Simulation.Output.INS.X_INS(1,4:6) = [0 0 0];
    Simulation.Output.INS.X_INS(1,7) = mean (Roll);
    Simulation.Output.INS.X_INS(1,8) = mean (Pitch);
    Simulation.Output.INS.X_INS(1,9) = mean (Yaw) + Simulation.Parameters_Misalignment.IMU_phins(3)*pi/180;
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %IMU
    if ~isempty(nonzeros(fb(:,1)))
        ave_fx = mean (fb(:,1));
    else
        ave_fx = 0;
    end 
    if ~isempty(nonzeros(fb(:,2)))
        ave_fy = mean (fb(:,2));
    else
        ave_fy = 0;
    end
    if ~isempty(nonzeros(fb(:,3)))
        ave_fz = mean (fb(:,3));
    else
        ave_fz = 0;
    end 
    ave_fb =[ave_fx ave_fy ave_fz];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ~isempty(nonzeros(W(:,1)))
        ave_Wx = mean (W(:,1));
    else
        ave_Wx = 0;
    end 
    if ~isempty(nonzeros(W(:,2)))
        ave_Wy = mean (W(:,2));
    else
        ave_Wy = 0;
    end
    if ~isempty(nonzeros(W(:,3)))
        ave_Wz = mean (W(:,3));
    else
        ave_Wz = 0;
    end    
    ave_W = [ave_Wx ave_Wy ave_Wz];  
    end


