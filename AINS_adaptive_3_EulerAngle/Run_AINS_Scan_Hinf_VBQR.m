% function [ Simulation ] = Run_AINS(Simulation,Real_Measurement,s_final,include_MA )
function [ Simulation ] = Run_AINS_Scan_Hinf_VBQR( )

%       load('Real_Measurement_simulateddata_LSTS_3', 'Real_Measurement')
%          load('Real_Measurement_testReal_LSTS_5Jun', 'Real_Measurement')
%         load('Parameter_stationary_5Jun_1Sam', 'Simulation') 
        
%         load('Real_Measurement_testReal_LSTS_145927_line_ms','Real_Measurement')
%         load('Parameter_stationary_testReal_LSTS_145927_line_ms','Simulation')

load('Real_Measurement_testReal_LSTS_AHRS_112509_quad_rpm_rev', 'Real_Measurement')
load('Parameter_stationary_testReal_LSTS_112509_quad_rpm_rev2','Simulation') 
  
% load('Real_Measurement_LSTS_Survey_113801_NP3_A44', 'Real_Measurement')
% load('Parameter_stationary_LSTS_Survey_113801_NP3_A44','Simulation')
 
% load('Real_Measurement_survy2_120233_A113-NP1','Real_Measurement')
% load('Real_Measurement_LSTS_Survey_113801_NP3_A44_cutted', 'Real_Measurement')
% load('Parameter_stationary_LSTS_Survey_113801_NP3_A44_1','Simulation')

% load('Parameter_stationary_LSTS_survy2_094337_A61-NP2', 'Simulation')
% load('Real_Measurement_survy2_094337_A61-NP2_Cutted','Real_Measurement')

% load('Real_Measurement_survy2_120233_A113-NP1','Real_Measurement')
% load('Parameter_stationary_testReal_LSTS_survy2_120233_A113-NP1', 'Simulation')

        Simulation.Input.Measurements.IMU       = Real_Measurement.IMU;
        Simulation.Input.Measurements.Heading   = Real_Measurement.Heading;
        Simulation.Input.Measurements.Ref_Pos   = Real_Measurement.Ref_Pos;
        Simulation.Input.Measurements.DVL       = Real_Measurement.DVL;
        Simulation.Input.Measurements.Depth     = Real_Measurement.Depth;
        Simulation.Input.Measurements.GPS       = Real_Measurement.GPS;
               
        coeff_MA = Simulation.Parameters_denoising.coeff_final;
        include_MA = Simulation.Parameters_denoising.include_MA; 
        
%         R_vd=[1e-2,2e-4,4e-4,6e-4,8e-4,1e-3,2e-5,4e-5,6e-5,8e-5,1e-4,2e-6,4e-6,6e-6,8e-6,1e-5];
        tau=[1,2,3,4,5,6,7,8,9,10];
        rho=[2,2.3,2.5,2.8,2.9,3,3.2,3.3,3.5,3.8,4];
        N=[1,2,3,4,5,6,7,8,9,10];
        gama2=[0.01,.1,.2,.3,.4,.5,.6,.7,.8,.9,1];

%     rej_vel=[100,200,400,600,800,1000,5000,10000,20000,30000,40000,50000,60000,70000,80000,90000,100000];
%     alpha=[1,10,100,1000];
%     rho =[0.5,0.7,0.9,1];
%         Dlen=+length(Qg)+length(Qba)+length(Qbg);
        k=1;

    for jj=1:length(N)
        for j=1:length(tau)
             for i=1:length(rho)
                for ii=1:length(gama2)
                Simulation.parameter_VB_adaptiveQR.tau=tau(j);
                Simulation.parameter_VB_adaptiveQR.rho = rho(i);
                Simulation.parameter_VB_adaptiveQR.N = N(jj);
                Simulation.parameter_HinfVB.gama2 =gama2(ii);
               
                
             Simulation = IINS_2(Simulation,coeff_MA,include_MA);
             scanParam=[tau(j);rho(i);N(jj);gama2(ii)];
             Result.scanParam(:,k) = scanParam; 
%              Result.R = R_vd(i);
%              Result.rej=rej_vel(jj);
%              Result.pos_m=Simulation.Output.ESKF.Pos_m;
%              Result.error=Simulation.Output.ESKF.Pos_Error;
             
%              Result.RMSE(:,k)=Simulation.Output.ESKF.Pos_Error.RMSE;
              Result.Relative_RMSE(k,1)= Simulation.Output.ESKF.Pos_Error.Relative_RMSE;
             Result.RMSE(k,1) =Simulation.Output.ESKF.Pos_Error.RMSE;
%              save(['ResultadaQ_' num2str(k) '.mat'],'Result')

%              filename= 'testdata.xlsx';
%              xlswrite(filename,A)
             k=k+1;
                end
             end
        end
    end
% for i=1:13
% load((['Result_' num2str(i) '.mat']))
% Resultadap.RelativeError(:,i)=Result.error.Relative_RMSE;
% % error(:,i)=Result.error.absolute_error;
% Resultadap.Q(i,:)=Result.Q;
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