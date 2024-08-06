%Transforation of the estimated position from radian(latitude,
%longitude and altitude) to meters and degree
%Pos_m: Estimated position in meters
%X_i  : Estimated position in radian
function [ Simulation ] = conversion( Simulation , ave_sample )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Constant Parameters
    %Equatorial radius
    a                  = 6378137;
    %eccentricity 
    e                  = 0.0818191908426;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      

        %$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        %$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
            
            Dlength=length(Simulation.Input.Measurements.IMU); %Dlength:The length of the data(time step)
            P0_geo=zeros(Dlength - ave_sample + 1,2); %P0_geo:Initial position in navigation frame in all timesteps
            RN=zeros(Dlength - ave_sample + 1,1);%Meridian radius of curvature
            RE=zeros(Dlength - ave_sample + 1,1); %Transverse radius of curvature        
            Simulation.Output.EKF.Pos_m=zeros(Dlength - ave_sample + 1,3);

                lat=Simulation.Output.EKF.X_i(:,1);
                for k=1:Dlength - ave_sample + 1
                    RN(k)=a*(1-e^2)/(1-e^2*(sin(lat(k)))^2)^1.5;
                    RE(k)=a/(1-e^2*(sin(lat(k)))^2)^0.5;
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                P0_geo(:,1)=Simulation.Output.EKF.X_i(1,1)*ones(Dlength - ave_sample + 1,1);
                P0_geo(:,2)=Simulation.Output.EKF.X_i(1,2)*ones(Dlength - ave_sample + 1,1);
        
                Simulation.Output.EKF.Pos_m(:,1)=(Simulation.Output.EKF.X_i(:,1)-P0_geo(:,1)).*RN;
                Simulation.Output.EKF.Pos_m(:,2)=(Simulation.Output.EKF.X_i(:,2)-P0_geo(:,2)).*cos(Simulation.Output.EKF.X_i(:,1)).*RE;
    
                Simulation.Output.EKF.Pos_m(:,3)=Simulation.Output.EKF.X_i(:,3);     
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                 
            %Average of the Estimated position in meter with respect to earth
            %surface in meter        
            Simulation.Output.EKF.ave_Pos_m=sum(Simulation.Output.EKF.Pos_m,3);            
            
            %Average of the estimated position expressed in the geographic frame in radian 
            ave_Pos_rad=sum(Simulation.Output.EKF.X_i,3);
            %Average of the estimated position expressed in the geographic
            %frame in degree
            Simulation.Output.EKF.ave_Pos_rad=zeros(Dlength,3);
            Simulation.Output.EKF.ave_Pos_rad(:,1) = ave_Pos_rad(:,1) ;
            Simulation.Output.EKF.ave_Pos_rad(:,2) = ave_Pos_rad(:,2) ;
            Simulation.Output.EKF.ave_Pos_rad(:,3) = ave_Pos_rad(:,3);%h=z             
        %$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        %$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
  

end

