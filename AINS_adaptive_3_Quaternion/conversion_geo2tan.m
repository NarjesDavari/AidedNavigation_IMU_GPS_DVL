function gps_m =conversion_geo2tan(GPS)
%      kalman_filter=kalman_filter2(gps_t3);
    a=6378137;%Equatorial radius
    e=0.081819190842621;%eccentricity

    
    Time=GPS(:,1);
    lat=GPS(:,2);
    lon=GPS(:,3);
    alt=GPS(:,4);
    Dlength=length(lat);
    
    RN=zeros(Dlength,1);
    RE=zeros(Dlength,1);    
    for i=1:Dlength
        RN(i)=a*(1-e^2)/(1-e^2*(sin(lat(i)*pi/180))^2)^1.5;
        RE(i)=a/(1-e^2*(sin(lat(i)*pi/180))^2)^0.5;
    end  
    
    gps.xn=zeros(Dlength,2);
    gps.xe=zeros(Dlength,2);
    gps.h=zeros(Dlength,2);
    
    lat0=lat(1)*ones(Dlength,1);
    lon0=lon(1)*ones(Dlength,1); 
    
    gps.xn(:,2)=(lat-lat0)*(pi/180).*RN;
    gps.xe(:,2)=(lon-lon0)*(pi/180).*cos(lat*pi/180).*RE;
    gps.h(:,2)=alt;  
    
    gps_m(:,1)=Time;
    gps_m(:,2)=gps.xn(:,2);
    gps_m(:,3)=gps.xe(:,2);
    gps_m(:,4)=gps.h(:,2);
    
end