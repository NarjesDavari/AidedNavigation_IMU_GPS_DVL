
% function []=ErrorEval(V_w)

% x = deadreckoning(V_w);


load('Real_Measuremet_test1_22th_2','Real_Measurement')
    a                  = 6378137;
    %eccentricity 
    e                  = 0.0818191908426;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Dlength:The length of the data(time step)
    x=Real_Measurement.GPS;
    Dlength=length(x) ; 
    %P0_geo:Initial position in navigation frame in all timesteps
    P0_geo=zeros(Dlength,2); 
        
    %Meridian radius of curvature
    RN=zeros(Dlength,1);
    %Transverse radius of curvature
    RE=zeros(Dlength ,1);        
    Pos_m=zeros(Dlength,3);
    lat=Real_Measurement.GPS(:,2)*pi/180;
    for k=1:Dlength 
        RN(k)=a*(1-e^2)/(1-e^2*(sin(lat(k)))^2)^1.5;
        RE(k)=a/(1-e^2*(sin(lat(k)))^2)^0.5;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    P0_geo(:,1)=Real_Measurement.GPS(1,2)*ones(Dlength ,1);
    P0_geo(:,2)=Real_Measurement.GPS(1,3)*ones(Dlength,1);
    Pos_m(:,1) = (Real_Measurement.GPS(:,2)-P0_geo(:,1))*pi/180.*RN;
    Pos_m(:,2) = (Real_Measurement.GPS(:,3)-P0_geo(:,2)).*cos(Real_Measurement.GPS(:,2))*pi/180.*RE;
    Pos_m(:,3) = Real_Measurement.GPS(:,4);%h=z        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                 

% end