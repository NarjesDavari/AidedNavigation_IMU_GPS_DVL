% function [ Simulation ] = Run_AINS(Simulation,Real_Measurement,s_final,include_MA )
function [ Simulation ] = Run_AINS_Scan2( )

% load('Real_Measurement_testReal_LSTS_112509_quad_rpm_rev', 'Real_Measurement')
% %  load('Real_Measurement_testReal_LSTS_112509_quad_rpm_rev_JustForTEST','Real_Measurement')
% load('Parameter_stationary_testReal_LSTS_112509_quad_rpm_rev','Simulation')
 

load('Real_Measurement_LSTS_Survey_113801_NP3_A44_cutted', 'Real_Measurement')
load('Parameter_stationary_LSTS_Survey_113801_NP3_A44_1','Simulation')
        
        Simulation.Input.Measurements.IMU       = Real_Measurement.IMU;
        Simulation.Input.Measurements.Heading   = Real_Measurement.Heading;
        Simulation.Input.Measurements.Ref_Pos   = Real_Measurement.Ref_Pos;
        Simulation.Input.Measurements.DVL       = Real_Measurement.DVL;
        Simulation.Input.Measurements.Depth     = Real_Measurement.Depth;
        Simulation.Input.Measurements.GPS       = Real_Measurement.GPS;
               
        coeff_MA = Simulation.Parameters_denoising.coeff_final;
        include_MA = Simulation.Parameters_denoising.include_MA; 
        
%         R_vd=[1e-2,2e-4,4e-4,6e-4,8e-4,1e-3,2e-5,4e-5,6e-5,8e-5,1e-4,2e-6,4e-6,6e-6,8e-6,1e-5];
   Qa=[1e-4,1e-5,1e-6,1e-7,1e-8,1e-9,1e-10];
        Qg=[1e-5,1e-6,1e-7,1e-8,1e-9,1e-10,1e-11];
        Qba=[1e-4,1e-5,1e-6,1e-7,1e-8,1e-9,1e-10];
        Qbg=[1e-10,1e-11,1e-12,1e-13,1e-14,1e-15];
%     rej_vel=[100,500,1000,5000,10000,20000,30000,40000,50000,60000,70000,80000,90000,100000];

k=1;
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
                
             Simulation = IINS_3(Simulation,coeff_MA,include_MA);

             
%              Result.pos_m=Simulation.Output.ESKF.Pos_m;
             Result.Q(k,:)=[Qa(j) Qba(i) Qba(jj) Qbg(ii)];
             Result.error(k,1)=Simulation.Output.ESKF.Pos_Error.Relative_RMSE;
%              
%              filename= 'testdata.xlsx';
%              xlswrite(filename,A)

             k=k+1;
            end
        end
    end
 end
%         save(['Result_' num2str(k) '.mat'],'Result')
% save(Result)
end
% for i=1:10
% load((['Result_' num2str(i) '.mat']))
% a(i,1)=Result.error.Relative_RMSE;
% end
% filename = 'testdata.xlsx';
% xlswrite(filename,a)