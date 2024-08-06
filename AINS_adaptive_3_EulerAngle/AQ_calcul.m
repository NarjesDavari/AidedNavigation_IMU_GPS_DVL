%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
%Computation of the transition matrix of the dynamic model(A) and the error
%covariance matrix(Q) in Error State Kalman Filter (ESKF) approach
%E:the system matrix(22x22)
%F:the system noise distribution matrix(22x6)
%x: corrected position and velocity vectors and the accel vector 
%C:corrected transform matrix
%x=[Lat_c,lon_c,h_c,Vn_c,Ve_c,Vh_c,fn,fe,fh]:15x1(N=15)
%R0:mean radius of the earth
%e:the major eccentricity of the ellipsoid
%Omega:Earth's rate
%Rm:the meridian radius of curvature
%Rt:the transverse radius of curvature
function [Simulation,flag_Qadapt]=AQ_calcul(x,C,Simulation,fs,I,Include_Q_adaptive,flag_Qadapt)

       
        q_ax=Simulation.Output.Kalman_mtx.Qc.q_ax;  
        q_ay=Simulation.Output.Kalman_mtx.Qc.q_ay; 
        q_az=Simulation.Output.Kalman_mtx.Qc.q_az; 
    
        q_wx=Simulation.Output.Kalman_mtx.Qc.q_wx;
        q_wy=Simulation.Output.Kalman_mtx.Qc.q_wy;
        q_wz=Simulation.Output.Kalman_mtx.Qc.q_wz;
        
        %% q_b, calculated from Bias instability of Allan Variance for GM
%         q_b_ax=2*Simulation.Output.Kalman_mtx.Qc.q_Bax/tau(1);
%         q_b_ay=2*Simulation.Output.Kalman_mtx.Qc.q_Bay/tau(2);
%         q_b_az=2*Simulation.Output.Kalman_mtx.Qc.q_Baz/tau(3);
%         
%         q_b_gx=2*Simulation.Output.Kalman_mtx.Qc.q_Bwx/tau(4);
%         q_b_gy=2*Simulation.Output.Kalman_mtx.Qc.q_Bwy/tau(5);
%         q_b_gz=2*Simulation.Output.Kalman_mtx.Qc.q_Bwz/tau(6);
     
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% q_b, calculated from Bias instability of Allan Variance for RW
        q_b_ax=Simulation.Output.Kalman_mtx.Qc.q_Bax;
        q_b_ay=Simulation.Output.Kalman_mtx.Qc.q_Bay;
        q_b_az=Simulation.Output.Kalman_mtx.Qc.q_Baz;
        
        q_b_gx=Simulation.Output.Kalman_mtx.Qc.q_Bwx;
        q_b_gy=Simulation.Output.Kalman_mtx.Qc.q_Bwy;
        q_b_gz=Simulation.Output.Kalman_mtx.Qc.q_Bwz;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        sigma_bias=[q_b_ax,q_b_ay,q_b_az,q_b_gx,q_b_gy,q_b_gz];
        
        [F,G]=FG_calcul(x,C);
        Simulation.Output.Kalman_mtx.F=0;
        Simulation.Output.Kalman_mtx.G=0;
        Simulation.Output.Kalman_mtx.F=F;
        Simulation.Output.Kalman_mtx.G=G;
        
        Qc=diag([q_ax,q_ay,q_az,q_wx,q_wy,q_wz,q_b_ax,q_b_ay,q_b_az,q_b_gx,q_b_gy,q_b_gz]);%%LxL(12X12)

%         %power spectral density of white noise process(W(t))(diagnal spectral density)
%         Qc=[q_ax  ,0     ,0     ,0     ,0     ,0      ;%LxL(6X6)
%             0     ,q_ay  ,0     ,0     ,0     ,0      ;
%             0     ,0     ,q_az  ,0     ,0     ,0      ;
%             0     ,0     ,0     ,q_wx  ,0     ,0      ;
%             0     ,0     ,0     ,0     ,q_wy  ,0      ;
%             0     ,0     ,0     ,0     ,0     ,q_wz  ];

        %Q:The covariance of the discrete process
        %A:the transition matrix of the dynamic model
        dt=1/fs;
        [A,Q] = lti_disc(F,G,Qc,dt);
        Simulation.Output.Kalman_mtx.A=0;
         Simulation.Output.Kalman_mtx.QQc=0;
        Simulation.Output.Kalman_mtx.A=A;
         Simulation.Output.Kalman_mtx.QQc=Qc;
%         Simulation.Output.Kalman_mtx.Q_diag(I,:)=[Q(1,1) Q(2,2) Q(3,3) Q(4,4) Q(5,5) Q(6,6) Q(7,7) Q(8,8) Q(9,9) Q(10,10) Q(11,11) Q(12,12) Q(13,13) Q(14,14) Q(15,15)];
%       Simulation.Output.User_Def_Sim.Kalman_mtx.norm_Q=0;
        Simulation.Output.Kalman_mtx.norm_Q(I,1)=norm(Q,2);
         if (Simulation.Output.Kalman_mtx.update_counter<Simulation.Output.Kalman_mtx.Number_adaptive && Include_Q_adaptive) || ~Include_Q_adaptive || I==1 || flag_Qadapt==0
        Simulation.Output.Kalman_mtx.Q=Q;
%         Simulation.Output.Kalman_mtx.Q_diag(I,:)=[Q(1,1) Q(2,2) Q(3,3) Q(4,4) Q(5,5) Q(6,6) Q(7,7) Q(8,8) Q(9,9) Q(10,10) Q(11,11) Q(12,12) Q(13,13) Q(14,14) Q(15,15)];
         end
         flag_Qadapt=0;
        
end  