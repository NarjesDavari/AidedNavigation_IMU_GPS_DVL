% function [ Simulation ] = Run_AINS(Simulation,Real_Measurement,s_final,include_MA )
function [ Simulation ] = Run_AINS_Scan_VB( )

%       load('Real_Measurement_simulateddata_LSTS_3', 'Real_Measurement')
%         load('Real_Measurement_testReal_LSTS_5Jun', 'Real_Measurement')
%         load('Parameter_stationary_5Jun_1Sam_adaptive', 'Simulation') 
load('Real_Measurement_testReal_LSTS_112509_quad_rpm_rev', 'Real_Measurement')
load('Parameter_stationary_adaptive_testReal_LSTS_112509_quad_rpm_rev','Simulation')
        

        
        Simulation.Input.Measurements.IMU       = Real_Measurement.IMU;
        Simulation.Input.Measurements.Heading   = Real_Measurement.Heading;
        Simulation.Input.Measurements.Ref_Pos   = Real_Measurement.Ref_Pos;
        Simulation.Input.Measurements.DVL       = Real_Measurement.DVL;
        Simulation.Input.Measurements.Depth     = Real_Measurement.Depth;
        Simulation.Input.Measurements.GPS       = Real_Measurement.GPS;
               
        coeff_MA = Simulation.Parameters_denoising.coeff_final;
        include_MA = Simulation.Parameters_denoising.include_MA; 
        
%         R_vd=[1e-2,2e-4,4e-4,6e-4,8e-4,1e-3,2e-5,4e-5,6e-5,8e-5,1e-4,2e-6,4e-6,6e-6,8e-6,1e-5];
%         Qa=[1e-4,1e-5,1e-6,1e-7,1e-8,1e-9];
%         Qg=[1e-6,1e-7,1e-8,1e-9,1e-10,1e-11];
%         Qba=[1e-5,1e-6,1e-7,1e-8,1e-9,1e-10];
%         Qbg=[1e-10,1e-11,1e-12,1e-13,1e-14,1e-15];
    alpha=[0.01,0.1,1,10,100,1000,2000];
    rho =[1,2,3,4,5,6,7,8,9,10];
%     Simulation.parameter_VB.rho=rho;
    N = [1,2,3,4,5,6,7];
        k=1;
     for jj=1:length(N)
        for i=1:length(rho)
            for j=1:length(alpha)
                   
                Simulation.parameter_VB.alpha=alpha(j);
                 Simulation.parameter_VB.rho =rho(i);
                Simulation.parameter_VB.N =N(jj);
                 
                 
             Simulation = IINS_2(Simulation,coeff_MA,include_MA);
            
             Result_adaptive.alpha = alpha(j); 
             Result_adaptive.rho = rho(i);
             Result_adaptive.N = N(jj);
             Result_adaptive.pos_m=Simulation.Output.ESKF.Pos_m;
             Result_adaptive.error=Simulation.Output.ESKF.Pos_Error;
             save(['Resultadaptive_' num2str(k) '.mat'],'Result_adaptive')

%              filename= 'testdata.xlsx';
%              xlswrite(filename,A)

             k=k+1;
            end
        end
     end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% k=1;
%          for jj=1:length(N)
% 
%                 for i=1:length(rho)
%                         for j=1:length(alpha)
% 
%                             ResultAdaptive.alpha(k,1) = alpha(j);
%                             ResultAdaptive.rho(k,1) = rho(i);
%                             ResultAdaptive.N(k,1)=N(jj);
%                             ResultAdaptive.error(k,1)=a(k,1);
%                             ResultAdaptive.absolute_error(:,k)=error(:,k);
%                             k=k+1;
%                         end
%                 end
%          end