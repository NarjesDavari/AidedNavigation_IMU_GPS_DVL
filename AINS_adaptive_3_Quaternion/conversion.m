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
    %Dlength:The length of the data(time step)
    Dlength=length(Simulation.Input.Measurements.IMU) ; 
    %P0_geo:Initial position in navigation frame in all timesteps
    P0_geo=zeros(Dlength - ave_sample + 1,2); 
        
    %Meridian radius of curvature
    RN=zeros(Dlength - ave_sample + 1,1);
    %Transverse radius of curvature
    RE=zeros(Dlength - ave_sample + 1,1);        
    Simulation.Output.ESKF.Pos_m=zeros(Dlength - ave_sample + 1,3);
    lat=Simulation.Output.INS.X_INS(:,1);
    for k=1:Dlength - ave_sample + 1
        RN(k)=a*(1-e^2)/(1-e^2*(sin(lat(k)))^2)^1.5;
        RE(k)=a/(1-e^2*(sin(lat(k)))^2)^0.5;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    P0_geo(:,1)=Simulation.Output.INS.X_INS(1,1)*ones(Dlength - ave_sample + 1,1);
    P0_geo(:,2)=Simulation.Output.INS.X_INS(1,2)*ones(Dlength- ave_sample + 1,1);
    Simulation.Output.ESKF.Pos_m(:,1) = (Simulation.Output.INS.X_INS(:,1)-P0_geo(:,1)).*RN;
    Simulation.Output.ESKF.Pos_m(:,2) = (Simulation.Output.INS.X_INS(:,2)-P0_geo(:,2)).*cos(Simulation.Output.INS.X_INS(:,1)).*RE;
    Simulation.Output.ESKF.Pos_m(:,3) = Simulation.Output.INS.X_INS(:,3);%h=z        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                 

end