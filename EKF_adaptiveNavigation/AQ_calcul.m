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
function [Simulation]=AQ_calcul(x,fb,Wib_b,C,Simulation,fs,I)


        [F,G]=FG_calcul_EKF(x,fb,Wib_b);

        Simulation.Output.Kalman_mtx.F=0;
        Simulation.Output.Kalman_mtx.G=0;
        Simulation.Output.Kalman_mtx.F=F;
        Simulation.Output.Kalman_mtx.G=G;
        %
        q_ax=Simulation.Output.Kalman_mtx.Qc.q_ax;  
        q_ay=Simulation.Output.Kalman_mtx.Qc.q_ay; 
        q_az=Simulation.Output.Kalman_mtx.Qc.q_az; 
    
        q_wx=Simulation.Output.Kalman_mtx.Qc.q_wx;
        q_wy=Simulation.Output.Kalman_mtx.Qc.q_wy;
        q_wz=Simulation.Output.Kalman_mtx.Qc.q_wz;
        
        q_bax=Simulation.Output.Kalman_mtx.Qc.q_Bax;
        q_bay=Simulation.Output.Kalman_mtx.Qc.q_Bay;
        q_baz=Simulation.Output.Kalman_mtx.Qc.q_Baz;
        
        q_bgx=Simulation.Output.Kalman_mtx.Qc.q_Bwx;
        q_bgy=Simulation.Output.Kalman_mtx.Qc.q_Bwy;
        q_bgz=Simulation.Output.Kalman_mtx.Qc.q_Bwz;

        %power spectral density of white noise process(W(t))(diagnal spectral density)
        Qc=diag([q_ax ,q_ay ,q_az ,q_wx,q_wy,q_wz,q_bax,q_bay,q_baz,q_bgx,q_bgy,q_bgz]);

        %Q:The covariance of the discrete process
        %A:the transition matrix of the dynamic model
        dt=1/fs;
        [A,Q] = lti_disc(F,G,Qc,dt);
        Simulation.Output.Kalman_mtx.A=0;
        Simulation.Output.Kalman_mtx.Q=0;
        Simulation.Output.Kalman_mtx.A=A;
        Simulation.Output.Kalman_mtx.Q=Q;
%         Simulation.Output.Kalman_mtx.Q_diag(I,:)=[Q(1,1) Q(2,2) Q(3,3) Q(4,4) Q(5,5) Q(6,6) Q(7,7) Q(8,8) Q(9,9)];
        
end  