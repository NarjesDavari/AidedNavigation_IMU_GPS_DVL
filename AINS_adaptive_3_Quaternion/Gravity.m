%Computation of local gravity vector
%x=[Lat,lon,h,Vn,Ve,Vd,fn,fe,fd,phi,theta,psi,Wx,Wy,Wz]:15x1(N=15)
%R:the length of the semi-major axis
%e:the major eccentricity of the ellipsoid
%Omega:Earth's rate
%gg:The mass attraction of the Earth
%RN:the meridian radius of curvature
%RE:the transverse radius of curvature
%R0:The mean radius of curvature
%Reference : My thesis page 41
function [g1,g0] = Gravity( x )

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Constant Parameters of the Earth
    R=6378137;%a:Equatorial radius
    e=0.0818191908426;
    Omega=7.292115e-05;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    RN=R*(1-e^2)/(1-e^2*(sin(x(1)))^2)^1.5;
    RE=R/(1-e^2*(sin(x(1)))^2)^0.5; 
    R0=sqrt(RN.*RE);   
    Wie=Omega;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    g0=9.78032*(1+5.3024e-3*(sin(x(1)))^2-5.9e-6*(sin(2*x(1)))^2);%g0=9.8157;%Lat:32.3887749733333
%     g0=9.8157;
    g_h=g0/(1+x(3)/R0)^2;%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     g = [0 0 g0];
    g=[0 0 g_h];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   g2=[sin(2*x(1)) 0 1+cos(2*x(1))];
    g2=[0 0 0];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     g1=g-((Wie^2)*(R0+x(3))/2)*g2;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% ooooooor
     g3=9.78032*(1+5.3024e-3*(sin(x(1)))^2-5.8e-6*(sin(2*x(1)))^2)-(3.0877e-6-4.4e-9*(sin(x(1)))^2)*x(3)+7.2e-14*(x(3))^2;
     g1=[0 0 g3];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end

