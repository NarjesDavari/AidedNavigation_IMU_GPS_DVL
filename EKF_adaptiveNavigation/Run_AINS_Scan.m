% function [ Simulation ] = Run_AINS(Simulation,Real_Measurement,s_final,include_MA )
function [ Simulation ] = Run_AINS_Scan( )

load('Real_Measurement_testReal_LSTS_112509_quad_rpm_rev', 'Real_Measurement')
% load('Parameter_stationary_testReal_LSTS_112509_quad_rpm_rev','Simulation')
load('Parameter_stationary_testReal_LSTS_112509_quad_rpm_rev','Simulation')
        
        
        Simulation.Input.Measurements.IMU       = Real_Measurement.IMU;
        Simulation.Input.Measurements.Heading   = Real_Measurement.Heading;
        Simulation.Input.Measurements.Ref_Pos   = Real_Measurement.Ref_Pos;
        Simulation.Input.Measurements.DVL       = Real_Measurement.DVL;
        Simulation.Input.Measurements.Depth     = Real_Measurement.Depth;
        Simulation.Input.Measurements.GPS       = Real_Measurement.GPS;
               
        coeff_MA = Simulation.Parameters_denoising.coeff_final;
        include_MA = Simulation.Parameters_denoising.include_MA; 
        
        R_vd=1e-2;      %         R_vd=[1e-2,2e-4,4e-4,6e-4,8e-4,1e-3,2e-5,4e-5,6e-5,8e-5,1e-4,2e-6,4e-6,6e-6,8e-6,1e-5];
        Qa=[1e-4,1e-5,1e-6,1e-7,1e-8,1e-9,1e-10];
        Qg=[1e-5,1e-6,1e-7,1e-8,1e-9,1e-10,1e-11];
        Qba=[1e-4,1e-5,1e-6,1e-7,1e-8,1e-9,1e-10];
        Qbg=[1e-13,1e-14,1e-15]; %%1e-9,1e-10,1e-11,1e-12,

%     rej_vel=[100,200,400,600,800,1000,5000,10000,20000,30000,40000,50000,60000,70000,80000,90000,100000];
%     alpha=[1,10,100,1000];
%     rho =[0.5,0.7,0.9,1];
%         Dlen=+length(Qg)+length(Qba)+length(Qbg);
        k=1;
for kk =1:length(R_vd)
     for ii=1:length(Qbg)
         for jj=1:length(Qba)
 
             for i=1:length(Qg)
                for j=1:length(Qa)
                    Simulation.Parameters_IMUNoisePSD.PSD_ax=Qa(j);
                    Simulation.Parameters_IMUNoisePSD.PSD_ay=Qa(j);
                    Simulation.Parameters_IMUNoisePSD.PSD_az=Qa(j);
                    Simulation.Parameters_IMUNoisePSD_Nav.PSD_ax=Simulation.Parameters_IMUNoisePSD.PSD_ax;
                    Simulation.Parameters_IMUNoisePSD_Nav.PSD_ay=Simulation.Parameters_IMUNoisePSD.PSD_ay;
                    Simulation.Parameters_IMUNoisePSD_Nav.PSD_az=Simulation.Parameters_IMUNoisePSD.PSD_az;

                    Simulation.Parameters_IMUNoisePSD.PSD_wx=Qg(i);
                    Simulation.Parameters_IMUNoisePSD.PSD_wy=Qg(i);
                    Simulation.Parameters_IMUNoisePSD.PSD_wz=Qg(i);
                    Simulation.Parameters_IMUNoisePSD_Nav.PSD_wx=Simulation.Parameters_IMUNoisePSD.PSD_wx;
                    Simulation.Parameters_IMUNoisePSD_Nav.PSD_wy=Simulation.Parameters_IMUNoisePSD.PSD_wy;
                    Simulation.Parameters_IMUNoisePSD_Nav.PSD_wz=Simulation.Parameters_IMUNoisePSD.PSD_wz;

                    Simulation.Parameters_IMUNoisePSD.PSD_Bax=Qba(jj);
                    Simulation.Parameters_IMUNoisePSD.PSD_Bay=Qba(jj);
                    Simulation.Parameters_IMUNoisePSD.PSD_Baz=Qba(jj);
                    Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bax=Simulation.Parameters_IMUNoisePSD.PSD_Bax;
                    Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bay=Simulation.Parameters_IMUNoisePSD.PSD_Bay;
                    Simulation.Parameters_IMUNoisePSD_Nav.PSD_Baz=Simulation.Parameters_IMUNoisePSD.PSD_Baz;

                    Simulation.Parameters_IMUNoisePSD.PSD_Bwx=Qbg(ii);
                    Simulation.Parameters_IMUNoisePSD.PSD_Bwy=Qbg(ii);
                    Simulation.Parameters_IMUNoisePSD.PSD_Bwz=Qbg(ii);
                    Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bwx=Simulation.Parameters_IMUNoisePSD.PSD_Bwx;
                    Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bwy=Simulation.Parameters_IMUNoisePSD.PSD_Bwy;
                    Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bwz=Simulation.Parameters_IMUNoisePSD.PSD_Bwz;

                    Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vx=R_vd(kk);
                    Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vy=R_vd(kk);
                    Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vz=R_vd(kk);
                    Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_alt=R_vd(kk);

                    Simulation.Parameters_AuxSnsrNoiseVar.var_vx=Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vx;
                    Simulation.Parameters_AuxSnsrNoiseVar.var_vy=Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vy;
                    Simulation.Parameters_AuxSnsrNoiseVar.var_vz=Simulation.Parameters_AuxSnsrNoiseVar_Nav.var_vz; 
    %                 
    %                  Simulation.parameter_VB.alpha=alpha(i)*ones(9,1);
    %                  Simulation.Output.Kalman_mtx.rho_VB=rho(i);
    %                   Simulation.Rej_Cof.VN=rej_vel(jj);
    %                   Simulation.Rej_Cof.VE=rej_vel(jj);
    %                   Simulation.Rej_Cof.VD=rej_vel(jj);

                 Simulation = IINS(Simulation);
                 Q=[Qa(j);Qg(i);Qba(jj);Qbg(ii)];
                 Result.Q = Q; 
                 Result.R = R_vd(kk);
    %              Result.rej=rej_vel(jj);
                 Result.pos_m=Simulation.Output.EKF.Pos_m;
                 Result.error=Simulation.Output.EKF.Pos_Error;
                 save(['ResultadaQ_' num2str(k) '.mat'],'Result')

    %              filename= 'testdata.xlsx';
    %              xlswrite(filename,A)

                 k=k+1;
                end
             end
         end
     end
end
% for i=1:13
% load((['Result_' num2str(i) '.mat']))
% Resultadap.RelativeError(:,i)=Result.error.Relative_RMSE;
% % error(:,i)=Result.error.absolute_error;
% Resultadap.Q(i,:)=Result.Q;
 % Resultadap.R(i,:)=Result.R
% end
% filename = 'testdata.xlsx';
% xlswrite(filename,a)
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% k=1;
%          for jj=1:length(rej_vel)
% 
%                 for i=1:length(R_vd)
%                         for j=1:length(Qa)
% 
%                             Result.Q(k,1:4) = [Qa(j);Qg(j);Qba(j);Qbg(j)];
%                             Result.R(k,1) = R_vd(i);
%                             Result.rej(k,1)=rej_vel(jj);
%                             Result.error(k,1)=a(k,1);
%                               Result.absolute_error(:,k)=error(:,k);
%                             k=k+1;
%                         end
%                 end
%          end
% %    