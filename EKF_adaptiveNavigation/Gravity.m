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
function gl = Gravity( x )

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Constant Parameters of the Earth
    R=6378137;
    e=0.0818191908426;
    Wie=7.292115e-05;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    RN=R*(1-e^2)/(1-e^2*(sin(x(1)))^2)^1.5;
    RE=R/(1-e^2*(sin(x(1)))^2)^0.5; 
    R0=sqrt(RN.*RE);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    g0=9.780318*(1+5.3024e-3*(sin(x(1))^2)-5.9e-6*(sin(2*x(1)))^2);
    gh=g0/(1+(x(3)/R0))^2;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    g=[0 0 gh];
%     g=[0 0 gg];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     g2=[sin(2*x(1)) 0 1+cos(2*x(1))];
    g2=[0 0 0];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    gl=g-((Wie^2)*(R0+x(3))/2)*g2;

end

