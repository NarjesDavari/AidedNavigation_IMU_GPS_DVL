
function [x_dot , fnn , Wnb_b] = Navigation_f( x_ , C_ , fb , Wib_b , gl )
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Constant Parameters of the Earth
    R=6378137;%meter
    e=0.0818191908426;
    Omega=7.292115e-05;%Earth's rate, rad/s
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    RN =R*(1-e^2)/(1-e^2*(sin(x_(1)))^2)^1.5;%meridian radius of curvature
    RE =R/(1-e^2*(sin(x_(1)))^2)^0.5;%transverse radius of curvature
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    SL       = sin(x_(1));
    CL       = cos(x_(1));
    tgL      = tan(x_(1));
    secL     = sec(x_(1));    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    q1 = x_(7);
    q2 = x_(8);
    q3 = x_(9);
    q4 = x_(10);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    inv_RN_z     = 1/(RN+x_(3));
    inv_RE_z     = 1/(RE+x_(3));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    WN = Omega * CL + x_(5) * inv_RE_z;
    WE = -x_(4) * inv_RN_z;
    WD = -Omega * SL - x_(5) * tgL * inv_RE_z;    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    fN = C_(1,1) * fb(1) + C_(1,2) * fb(2) + C_(1,3) * fb(3);
    fE = C_(2,1) * fb(1) + C_(2,2) * fb(2) + C_(2,3) * fb(3);
    fD = C_(3,1) * fb(1) + C_(3,2) * fb(2) + C_(3,3) * fb(3);
    
    fnn = [fN fE fD]; 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Wx = Wib_b(1) - (C_(1,1) * WN + C_(2,1) * WE + C_(3,1) * WD);
    Wy = Wib_b(2) - (C_(1,2) * WN + C_(2,2) * WE + C_(3,2) * WD);
    Wz = Wib_b(3) - (C_(1,3) * WN + C_(2,3) * WE + C_(3,3) * WD);
    
    Wnb_b =[Wx Wy Wz];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    x_dot = zeros(size(x_'));
    x_dot(1,:) = x_(4) * inv_RN_z;
    x_dot(2,:) = x_(5) * secL * inv_RE_z;
    x_dot(3,:) = -x_(6);
    
    x_dot(4,:) = fN - x_(5) * (2 * Omega + x_dot(2,:)) * SL + x_(6) * x_dot(1,:);
    x_dot(5,:) = fE + x_(4) * (2 * Omega + x_dot(2,:)) * SL + x_(6) * (2 * Omega + x_dot(2,:)) * CL;
    x_dot(6,:) = fD - x_(5) * (2 * Omega + x_dot(2,:)) * CL - x_(4) * x_dot(1,:) + gl(3);

    x_dot(7,:) = 0.5*(Wx * q4 - Wy * q3 + Wz * q2);
    x_dot(8,:) = 0.5*(Wx * q3 + Wy * q4 - Wz * q1);
    x_dot(9,:) = 0.5*(-Wx * q2 + Wy * q1 + Wz *q4);
    x_dot(10,:) = 0.5*(-Wx * q1 - Wy * q2 - Wz *q3);
    
%     x_dot(11:16,:)=0;
    
%     Tca = 100; Tcg=100;
%     x_dot(10,:) = -1/Tca * x_(10);
%     x_dot(11,:) = -1/Tca * x_(11);
%     x_dot(12,:) = -1/Tca * x_(12);
%     x_dot(13,:) = -1/Tcg * x_(13);
%     x_dot(14,:) = -1/Tcg * x_(14);
%     x_dot(15,:) = -1/Tcg * x_(15);
end

