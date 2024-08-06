%Computation of coriolis effect on the velocity equations
%Reference: My thesis page 36-42
function W_Coriolis = Coriolis_correction( x )

    %x=[Lat,lon,h,Vn,Ve,Vd,phi,theta,psi,Bax,Bay,Baz,BWx,BWy,BWz]:15x1(N=15)
    %R:the length of the semi-major axis
    %e:the major eccentricity of the ellipsoid
    %Omega:Earth's rate
    %RN:the meridian radius of curvature
    %RE:the transverse radius of curvature
    %Wen:the turn rate of the Navigation frame with respect to the ECEF
    %Wie:the turn rate of the Earth expressed in the local geographic frame
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Constant Parameters
    R=6378137;
    e=0.0818191908426;
    Omega=7.292115e-05;  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    RN=R*(1-e^2)/(1-e^2*(sin(x(1)))^2)^1.5;
    RE=R/(1-e^2*(sin(x(1)))^2)^0.5;
    Wie=Omega;
    Wen_n=[x(5)/(RE+x(3)) -x(4)/(RN+x(3)) (-x(5)*tan(x(1)))/(RN+x(3))]';
    Wie_n=[Wie*cos(x(1)) 0 -Wie*sin(x(1))]';
    %
    W_Coriolis=2*Wie_n+Wen_n;

end

