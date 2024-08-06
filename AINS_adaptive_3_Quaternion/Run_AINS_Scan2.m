% function [ Simulation ] = Run_AINS(Simulation,Real_Measurement,s_final,include_MA )
function [ Simulation ] = Run_AINS_Scan2( )

%       load('Real_Measurement_simulateddata_LSTS_3', 'Real_Measurement')
        load('Real_Measurement_testReal_LSTS_5Jun', 'Real_Measurement')
        load('Parameter_stationary_5Jun_1Sam_adaptive', 'Simulation') 
        
        
        Simulation.Input.Measurements.IMU       = Real_Measurement.IMU;
        Simulation.Input.Measurements.Heading   = Real_Measurement.Heading;
        Simulation.Input.Measurements.Ref_Pos   = Real_Measurement.Ref_Pos;
        Simulation.Input.Measurements.DVL       = Real_Measurement.DVL;
        Simulation.Input.Measurements.Depth     = Real_Measurement.Depth;
        Simulation.Input.Measurements.GPS       = Real_Measurement.GPS;
               
        coeff_MA = Simulation.Parameters_denoising.coeff_final;
        include_MA = Simulation.Parameters_denoising.include_MA; 
        
%         R_vd=[1e-2,2e-4,4e-4,6e-4,8e-4,1e-3,2e-5,4e-5,6e-5,8e-5,1e-4,2e-6,4e-6,6e-6,8e-6,1e-5];
        Qa=[1e4,1e5,1e6,1e7];
%         Qg=[1e-6,1e-7,1e-8,1e-9,1e-10,1e-11];
        Qba=[1e3,1e4,1e5,1e6,1e7,1e8,1e9,1e10];
%         Qbg=[1e-10,1e-11,1e-12,1e-13,1e-14,1e-15];
%     rej_vel=[100,500,1000,5000,10000,20000,30000,40000,50000,60000,70000,80000,90000,100000];

k=1;
        for i=1:length(Qba)
            for j=1:length(Qa)
                 Simulation.Parameters_IMUNoisePSD.PSD_ax=Qa(j);
                Simulation.Parameters_IMUNoisePSD.PSD_ay=Qa(j);
                Simulation.Parameters_IMUNoisePSD.PSD_az=Qa(j);
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_ax=Simulation.Parameters_IMUNoisePSD.PSD_ax;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_ay=Simulation.Parameters_IMUNoisePSD.PSD_ay;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_az=Simulation.Parameters_IMUNoisePSD.PSD_az;
                
                Simulation.Parameters_IMUNoisePSD.PSD_wx=Qa(j);
                Simulation.Parameters_IMUNoisePSD.PSD_wy=Qa(j);
                Simulation.Parameters_IMUNoisePSD.PSD_wz=Qa(j);
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_wx=Simulation.Parameters_IMUNoisePSD.PSD_wx;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_wy=Simulation.Parameters_IMUNoisePSD.PSD_wy;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_wz=Simulation.Parameters_IMUNoisePSD.PSD_wz;
                
                Simulation.Parameters_IMUNoisePSD.PSD_Bax=Qba(i);
                Simulation.Parameters_IMUNoisePSD.PSD_Bay=Qba(i);
                Simulation.Parameters_IMUNoisePSD.PSD_Baz=Qba(i);
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bax=Simulation.Parameters_IMUNoisePSD.PSD_Bax;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bay=Simulation.Parameters_IMUNoisePSD.PSD_Bay;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_Baz=Simulation.Parameters_IMUNoisePSD.PSD_Baz;
                
                Simulation.Parameters_IMUNoisePSD.PSD_Bwx=Qba(i);
                Simulation.Parameters_IMUNoisePSD.PSD_Bwy=Qba(i);
                Simulation.Parameters_IMUNoisePSD.PSD_Bwz=Qba(i);
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bwx=Simulation.Parameters_IMUNoisePSD.PSD_Bwx;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bwy=Simulation.Parameters_IMUNoisePSD.PSD_Bwy;
                Simulation.Parameters_IMUNoisePSD_Nav.PSD_Bwz=Simulation.Parameters_IMUNoisePSD.PSD_Bwz;
                
             Simulation = IINS_2(Simulation,coeff_MA,include_MA);

             
%              Result.pos_m=Simulation.Output.ESKF.Pos_m;
             Result.Q(k,:)=[Qa(j) Qba(i)];
             Result.error(k,1)=Simulation.Output.ESKF.Pos_Error.Relative_RMSE;
%              
%              filename= 'testdata.xlsx';
%              xlswrite(filename,A)

             k=k+1;
            end
        end
%         save(['Result_' num2str(k) '.mat'],'Result')
save(Result)
end
% for i=1:10
% load((['Result_' num2str(i) '.mat']))
% a(i,1)=Result.error.Relative_RMSE;
% end
% filename = 'testdata.xlsx';
% xlswrite(filename,a)