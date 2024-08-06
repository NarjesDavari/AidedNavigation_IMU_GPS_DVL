%Computation of the system matrix (F) and the system noise distribution
%matrix (G)
%x_dot=f(x,u,w)~=Fx+Gw
%x=[L l z Vn Ve Vd roll pitch yaw]T
%u=[fx fy fz wx wy wz]T;
%w=[dfx dfy dfz dwx dwy dwz]T
%C=CBtoN
%Reference : My thesis page 88-89
%            Titterton page 344            
function [ F,G ] = FG_calcul_EKF( x,fb,Wib_b )
    %x=[Lat;lon;d;Vn;Ve;Vd;fn;fe;fd]
    %C=body to navigation
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Constant Parameters of the Earth
    R=6378137;%meter
    e=0.0818191908426;
    Omega=7.292115e-05;%Earth's rate, rad/s
%     gg=9.782421625992459;%gg:Mass attraction of the Earth
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    RN =R*(1-e^2)/(1-e^2*(sin(x(1)))^2)^1.5;%meridian radius of curvature
    RE =R/(1-e^2*(sin(x(1)))^2)^0.5;%transverse radius of curvature
    R0=sqrt(RN*RE);%mean radius of the earth
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    SL       = sin(x(1));
    CL       = cos(x(1));
    tgL      = tan(x(1));
    secL     = sec(x(1));
    sqr_secL = sec(x(1)) ^ 2; %1 + tanL^2
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    SPhi      = sin(x(7));
    CPhi      = cos(x(7));
    STheta    = sin(x(8));
    CTheta    = cos(x(8));
    SPsi      = sin(x(9));
    CPsi      = sin(x(9));
    tgTheta   = tan(x(8));
    secTheta  = sec(x(8));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    d_RN_d_L = 1.5 * e^2 * sin(2*x(1)) * R * (1 - e^2) * (1-(e^2)*(SL^2))^(-2.5);
    d_RE_d_L = 0.5 * e^2 * sin(2*x(1)) * (1-(e^2)*(SL^2))^(-1.5);        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    inv_RN_z     = 1/(RN+x(3));
    inv_RE_z     = 1/(RE+x(3));
    inv_sqr_RN_z = 1/(RN+x(3))^2;
    inv_sqr_RE_z = 1/(RE+x(3))^2;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    WN = Omega * CL + x(5) * inv_RE_z;
    WE = -x(4) * inv_RN_z;
    WD = -Omega * SL - x(5) * tgL * inv_RE_z;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    C11 =  cos(x(8))*cos(x(9));
    C12 = -cos(x(7))*sin(x(9)) + sin(x(7))*sin(x(8))*cos(x(9));
    C13 =  sin(x(7))*sin(x(9)) + cos(x(7))*sin(x(8))*cos(x(9));
    C21 =  cos(x(8))*sin(x(9));
    C22 =  cos(x(7))*cos(x(9)) + sin(x(7))*sin(x(8))*sin(x(9));
    C23 = -sin(x(7))*cos(x(9)) + cos(x(7))*sin(x(8))*sin(x(9));
    C31 = -sin(x(8));
    C32 =  sin(x(7))*cos(x(8));
    C33 =  cos(x(7))*cos(x(8));  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    Wx = Wib_b(1) - (C11 * WN + C21 * WE + C31 * WD);
    Wy = Wib_b(2) - (C12 * WN + C22 * WE + C32 * WD);
    Wz = Wib_b(3) - (C13 * WN + C23 * WE + C33 * WD);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    dC11_dPhi = 0;
    dC12_dPhi = SPhi * SPsi + CPhi * STheta * CPsi;
    dC13_dPhi = CPhi * SPsi - SPhi * STheta * CPsi;
    dC11_dTheta = -STheta * CPsi;
    dC12_dTheta = SPhi * CTheta * CPsi;
    dC13_dTheta = CPhi * CTheta * CPsi;  
    dC11_dPsi = -CTheta * SPsi;
    dC12_dPsi = -CPhi * CPsi - SPhi * STheta * SPsi;
    dC13_dPsi = SPhi * CPsi - CPhi * STheta * SPsi;  
    
    dC21_dPhi = 0;
    dC22_dPhi = -SPhi * CPsi + CPhi * STheta * SPsi;
    dC23_dPhi = -CPhi * CPsi - SPhi * STheta * SPsi;
    dC21_dTheta = -STheta * SPsi;
    dC22_dTheta = SPhi * CTheta * SPsi;
    dC23_dTheta = CPhi * CTheta * SPsi;  
    dC21_dPsi = CTheta * CPsi;
    dC22_dPsi = -CPhi * SPsi + SPhi * STheta * CPsi;
    dC23_dPsi = SPhi * SPsi + CPhi * STheta * CPsi;
    
    dC31_dPhi = 0;
    dC32_dPhi = CPhi * CTheta;
    dC33_dPhi = -SPhi * CTheta;
    dC31_dTheta = -CTheta;
    dC32_dTheta = -SPhi * STheta;
    dC33_dTheta = -CPhi * STheta;  
    dC31_dPsi = 0;
    dC32_dPsi = 0;
    dC33_dPsi = 0;    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    dWN_dL  = - Omega * SL - d_RE_d_L * x(5) * inv_sqr_RE_z;
    %dWN_dl = 0;
    dWN_dz  = -x(5) * inv_sqr_RE_z;
    %dWN_dVN = 0;
    dWN_dVE = inv_RE_z;
    %dWN_dVD = 0;
    
    dWE_dL  = d_RN_d_L * x(4) * inv_sqr_RN_z;
    %dWE_dl = 0;
    dWE_dz  = x(4) * inv_sqr_RN_z;
    dWE_dVN = -inv_RN_z;
    %dWE_dVE = 0;
    %dWE_dVD = 0;
    
    dWD_dL  = d_RE_d_L * x(5) * tgL * inv_sqr_RE_z - Omega * CL - x(5) * sqr_secL * inv_RE_z;
    %dWD_dl = 0;
    dWD_dz  = x(5) * tgL * inv_sqr_RE_z;
    %dWD_dVN = 0;
    dWD_dVE = -tgL * inv_RE_z;
    %dWD_dVD = 0;     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    dWx_dL     = -(C11 * dWN_dL + C21 * dWE_dL + C31 * dWD_dL);
    dWx_dl     = 0;
    dWx_dz     = -(C11 * dWN_dz + C21 * dWE_dz + C31 * dWD_dz);
    dWx_dVN    = C21 * dWE_dVN;
    dWx_dVE    = -(C11 * dWN_dVE + C31 * dWD_dVE);
    dWx_dVD    = 0;
    dWx_dPhi   = 0;
    dWx_dTheta = -(dC11_dTheta * WN + dC21_dTheta * WE + dC31_dTheta * WD);
    dWx_dPsi   = -(dC11_dPsi * WN + dC21_dPsi * WE + dC31_dPsi * WD);
    
    dWy_dL     = -(C12 * dWN_dL + C22 * dWE_dL + C32 * dWD_dL);
    dWy_dl     = 0;
    dWy_dz     = -(C12 * dWN_dz + C22 * dWE_dz + C32 * dWD_dz);
    dWy_dVN    = C22 * dWE_dVN;
    dWy_dVE    = -(C12 * dWN_dVE + C32 * dWD_dVE);
    dWy_dVD    = 0;
    dWy_dPhi   = -(dC12_dPhi * WN + dC22_dPhi * WE + dC32_dPhi * WD);
    dWy_dTheta = -(dC12_dTheta * WN + dC22_dTheta * WE + dC32_dTheta * WD);
    dWy_dPsi   = -(dC12_dPsi * WN + dC22_dPsi * WE + dC32_dPsi * WD);
    
    dWz_dL     = -(C13 * dWN_dL + C23 * dWE_dL + C33 * dWD_dL);
    dWz_dl     = 0;
    dWz_dz     = -(C13 * dWN_dz + C23 * dWE_dz + C33 * dWD_dz);
    dWz_dVN    = C23 * dWE_dVN;
    dWz_dVE    = -(C13 * dWN_dVE + C33 * dWD_dVE);
    dWz_dVD    = 0;
    dWz_dPhi   = -(dC13_dPhi * WN + dC23_dPhi * WE + dC33_dPhi * WD);
    dWz_dTheta = -(dC13_dTheta * WN + dC23_dTheta * WE + dC33_dTheta * WD);
    dWz_dPsi   = -(dC13_dPsi * WN + dC23_dPsi * WE + dC33_dPsi * WD);    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    F11 = -d_RN_d_L * x(4) * inv_sqr_RN_z;    
    F12 = 0;
    F13 = -x(4) * inv_sqr_RN_z;
    F14 = inv_RN_z;
    F15 = 0; F16 = 0; F17 = 0; F18 = 0; F19 = 0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    F21 = x(5) * (secL * tgL * inv_RE_z  - d_RE_d_L * secL * inv_sqr_RE_z); %%%%%%%%%%%%%F21 = x(5) * (secL * tgL * inv_RE_z  - d_RE_d_L * secL * inv_sqr_RE_z)
    F22 = 0;
    F23 = -x(5) * secL * inv_sqr_RE_z;
    F24 = 0;
    F25 = secL * inv_RE_z;
    F26 = 0; F27 = 0; F28 = 0; F29 = 0;    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    F31 = 0; F32 = 0; F33 = 0;
    F34 = 0; F35 = 0; F36 = 1;
    F37 = 0; F38 = 0; F39 = 0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    F41 = -x(5) * ( 2 * Omega * CL + F21 * SL + x(5) * inv_RE_z) + x(6) * F11;
    F42 = 0;
    F43 = -x(5) * SL * F23 + x(6) * F13;
    F44 = x(6) * F14;
    F45 = -2 * Omega * SL - 2 * x(5) * tgL * inv_RE_z;
    F46 = x(4) * inv_RN_z;
    F47 = dC11_dPhi * fb(1) + dC12_dPhi * fb(2) + dC13_dPhi * fb(3);
    F48 = dC11_dTheta * fb(1) + dC12_dTheta * fb(2) + dC13_dTheta * fb(3);
    F49 = dC11_dPsi * fb(1) + dC12_dPsi * fb(2) + dC13_dPsi * fb(3);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    F51 = x(4) * ( 2 * Omega * CL + F21 * SL + x(5) * inv_RE_z * tgL) + x(6) * (-2 * Omega * SL + F21 * CL + x(5) * inv_RE_z);
    F52 = 0;
    F53 = F23 *(x(4)*SL + x(6)*CL);
    F54 = (2 * Omega + x(5) * secL * inv_RE_z)*SL;
    F55 = (x(6) + x(4) * tgL) * inv_RE_z;
    F56 = (2 * Omega + x(5) * secL * inv_RE_z)*CL;
    F57 = dC21_dPhi   * fb(1) + dC22_dPhi   * fb(2) + dC23_dPhi   * fb(3);
    F58 = dC21_dTheta * fb(1) + dC22_dTheta * fb(2) + dC23_dTheta * fb(3);
    F59 = dC21_dPsi   * fb(1) + dC22_dPsi   * fb(2) + dC23_dPsi   * fb(3);    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    F61 = -x(5) * ( -2 * Omega * SL + F21 * CL - x(5) * inv_RE_z * tgL) - x(4) * F11;
    F62 = 0;
    F63 = -x(4) * F13 - x(5) * CL * F23;
    F64 = -2 * x(4) * inv_RN_z;
    F65 = -2 * Omega * CL - 2 * x(5) * inv_RE_z;
    F66 = 0;
    F67 = dC31_dPhi   * fb(1) + dC32_dPhi   * fb(2) + dC33_dPhi   * fb(3);
    F68 = dC31_dTheta * fb(1) + dC32_dTheta * fb(2) + dC33_dTheta * fb(3);
    F69 = dC31_dPsi   * fb(1) + dC32_dPsi   * fb(2) + dC33_dPsi   * fb(3);    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    F71 = (dWy_dL * SPhi + dWz_dL * CPhi) * tgTheta + dWx_dL;
    F72 = 0;
    F73 = (dWy_dz * SPhi + dWz_dz * CPhi) * tgTheta + dWx_dz;
    F74 = (dWy_dVN * SPhi + dWz_dVN * CPhi) * tgTheta + dWx_dVN;
    F75 = (dWy_dVE * SPhi + dWz_dVE * CPhi) * tgTheta + dWx_dVE;
    F76 = (dWy_dVD * SPhi + dWz_dVD * CPhi) * tgTheta + dWx_dVD;
    F77 = (dWy_dPhi * SPhi + Wy * CPhi + dWz_dPhi * CPhi - Wz * SPhi) * tgTheta + dWx_dPhi;
    F78 = (dWy_dTheta * SPhi + dWz_dTheta * CPhi) * tgTheta + (Wy * SPhi + Wz * CPhi) * (secTheta^2) + dWx_dTheta;
    F79 = (dWy_dPsi * SPhi + dWz_dPsi * CPhi) * tgTheta + dWx_dPsi;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    F81 = dWy_dL * CPhi - dWz_dL * SPhi;
    F82 = 0;
    F83 = dWy_dz * CPhi - dWz_dz * SPhi;
    F84 = dWy_dVN * CPhi - dWz_dVN * SPhi;
    F85 = dWy_dVE * CPhi - dWz_dVE * SPhi;
    F86 = dWy_dVD * CPhi - dWz_dVD * SPhi;
    F87 = dWy_dPhi * CPhi - Wy * SPhi - dWz_dPhi * SPhi - Wz * CPhi;
    F88 = dWy_dTheta * CPhi - dWz_dTheta * SPhi;
    F89 = dWy_dPsi * CPhi - dWz_dPsi * SPhi;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    F91 = (dWy_dL * SPhi + dWz_dL * CPhi) * secTheta;
    F92 = 0;
    F93 = (dWy_dz * SPhi + dWz_dz * CPhi) * secTheta;
    F94 = (dWy_dVN * SPhi + dWz_dVN * CPhi) * secTheta;
    F95 = (dWy_dVE * SPhi + dWz_dVE * CPhi) * secTheta;
    F96 = (dWy_dVD * SPhi + dWz_dVD * CPhi) * secTheta;
    F97 = (dWy_dPhi * SPhi + Wy * CPhi + dWz_dPhi * CPhi - Wz * CPhi) * secTheta;
    F98 = (dWy_dTheta * SPhi + dWz_dTheta * CPhi) * secTheta + (Wy * SPhi + Wz * CPhi) * secTheta * tgTheta;
    F99 = (dWy_dPsi * SPhi + dWz_dPsi * CPhi) * secTheta;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    C_bn=[C11,C12,C13;C21,C22,C23;C31,C32,C33];
    F_b2 =[1 0 0; 0 CPhi^2 SPhi^2; 0 SPhi^2 CPhi^2];
    F_b=[zeros(3,6);-C_bn,zeros(3);zeros(3),-F_b2];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    F1 = [F11 , F12 , F13 , F14 , F15 , F16 , F17 , F18 , F19 
         F21 , F22 , F23 , F24 , F25 , F26 , F27 , F28 , F29
         F31 , F32 , F33 , F34 , F35 , F36 , F37 , F38 , F39
         F41 , F42 , F43 , F44 , F45 , F46 , F47 , F48 , F49
         F51 , F52 , F53 , F54 , F55 , F56 , F57 , F58 , F59
         F61 , F62 , F63 , F64 , F65 , F66 , F67 , F68 , F69
         F71 , F72 , F73 , F74 , F75 , F76 , F77 , F78 , F79
         F81 , F82 , F83 , F84 , F85 , F86 , F87 , F88 , F89
         F91 , F92 , F93 , F94 , F95 , F96 , F97 , F98 , F99];%9x9
F = [F1 F_b; zeros(6,15)];
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   G11 = 0; G12 = 0; G13 = 0; G14 = 0; G15 = 0; G16 = 0;
   G21 = 0; G22 = 0; G23 = 0; G24 = 0; G25 = 0; G26 = 0;
   G31 = 0; G32 = 0; G33 = 0; G34 = 0; G35 = 0; G36 = 0;
   
   G41 = C11; G42 = C12; G43 = C13; G44 = 0; G45 = 0; G46 = 0;
   G51 = C21; G52 = C22; G53 = C23; G54 = 0; G55 = 0; G56 = 0;
   G61 = C31; G62 = C32; G63 = C33; G64 = 0; G65 = 0; G66 = 0;
   
   G71 = 0; G72 = 0; G73 = 0; G74 = 1; G75 = SPhi * tgTheta; G76 = CPhi * tgTheta;
   G81 = 0; G82 = 0; G83 = 0; G84 = 0; G85 = CPhi;           G86 = -SPhi;
   G91 = 0; G92 = 0; G93 = 0; G94 = 0; G95 = SPhi * secTheta; G96 = CPhi * secTheta;
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    G1=[G11 G12 G13 G14 G15 G16
%       G21 G22 G23 G24 G25 G26
%       G31 G32 G33 G34 G35 G36
%       G41 G42 G43 G44 G45 G46
%       G51 G52 G53 G54 G55 G56
%       G61 G62 G63 G64 G65 G66
%       G71 G72 G73 G74 G75 G76
%       G81 G82 G83 G84 G85 G86
%       G91 G92 G93 G94 G95 G96];%9x6
%   G =[G1,-G1;zeros(6),eye(6)];
  
     C = [C11 C12 C13
        C21 C22 C23
        C31 C32 C33];
   G_1 =[G74 G75 G76
         G84 G85 G86
         G94 G95 G96];
    G=[zeros(3,12)
       C          , zeros(3) , C , zeros(3)
       zeros(3)   , G_1         , zeros(3)  , G_1   
       zeros(3,6) , eye(3)    , zeros(3)
       zeros(3,9) , eye(3)];%15x12
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

end