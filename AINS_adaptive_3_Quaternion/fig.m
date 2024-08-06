figure;plot(Simulation.Input.Measurements.Ref_Pos(1:end,3),Simulation.Input.Measurements.Ref_Pos(1:end,2))
hold on
plot(Simulation.Output.ESKF.Pos_m(:,2),Simulation.Output.ESKF.Pos_m(:,1),'r')

figure;plot(Simulation.Input.Measurements.IMU(:,1),Simulation.Output.ESKF.Pos_Error.absolute_error)

for i=1:3
  figure;plot(Simulation_3.Input.Measurements.IMU(:,1),Simulation_3.Input.Measurements.Ref_Pos(1:end,i+1))
hold on
plot(Simulation_2.Input.Measurements.IMU(:,1),Simulation_2.Output.ESKF.Pos_m(:,i),'r')
hold on
plot(Simulation_3.Input.Measurements.IMU(:,1),Simulation_3.Output.ESKF.Pos_m(:,i),'r')
end
figure;subplot(2,1,1);plot(Simulation.Input.Measurements.Depth(:,2))
subplot(2,1,2);plot(Simulation_6.Output.Kalman_mtx.R_adaptive.R_Depth)

figure;plot(Real_Measurement.Heading_interp(:,2))
hold on;plot(Simulation.Output.ESKF.O_corrected(:,9)*180/pi,'r')

figure
plot(Simulation.Output.ESKF.O_corrected(:,13:15))

figure;plot(Simulation.Output.Kalman_mtx.P_diag(:,13:15))


for i=1:3
    figure;
subplot(2,1,1);plot(ans.Input.Measurements.DVL(:,1),ans.Input.Measurements.DVL(:,i+1))
ylabel('Velocity (m/Sec)');
subplot(2,1,2);plot(ans.Input.Measurements.DVL(:,1),ans.Output.Kalman_mtx.R_adaptive.R_DVL(:,i),'r')
xlabel('Time (s)');
ylabel('R of V_{x} (m/Sec)^2');

end

legend('KFNCM','KFTCM','The proposed filter');

figure;plot(nonzeros(Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(:,1)))
figure;plot(nonzeros(Simulation.Output.Kalman_mtx.R_adaptive.R_GPS(:,2)))

figure;plot(Simulation_2.Input.Measurements.IMU(:,1),Simulation.Output.ESKF.Pos_Error.absolute_error);hold on;
plot(Simulation_2.Input.Measurements.IMU(:,1),Simulation.Output.ESKF.Pos_Error.absolute_error,'r')
Simulation.Output.ESKF.Pos_Error.Relative_RMSE

a1=20000;a2=.0001;a3=20000;a4=1;
Simulation.Parameters_IMUNoisePSD.TC_qax=.1*a1;
Simulation.Parameters_IMUNoisePSD.TC_qay=3*a1;
Simulation.Parameters_IMUNoisePSD.TC_qaz=.01*a1;
Simulation.Parameters_IMUNoisePSD.TC_qwx=2*a2;
Simulation.Parameters_IMUNoisePSD.TC_qwy=5*a2;
Simulation.Parameters_IMUNoisePSD.TC_qwz=0.1*a2;
Simulation.Parameters_IMUNoisePSD.TC_qBax=.1*a3;
Simulation.Parameters_IMUNoisePSD.TC_qBay=3*a3;
Simulation.Parameters_IMUNoisePSD.TC_qBaz=.01*a3;
Simulation.Parameters_IMUNoisePSD.TC_qBwx=2*a4;
Simulation.Parameters_IMUNoisePSD.TC_qBwy=5*a4;
Simulation.Parameters_IMUNoisePSD.TC_qBwz=.1*a4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% for test 1
a1=200;a2=.1;a3=200;a4=.1;
Simulation.Parameters_IMUNoisePSD.TC_qax=.01*a1;
Simulation.Parameters_IMUNoisePSD.TC_qay=500*a1;
Simulation.Parameters_IMUNoisePSD.TC_qaz=.01*a1;
Simulation.Parameters_IMUNoisePSD.TC_qwx=5*a2;
Simulation.Parameters_IMUNoisePSD.TC_qwy=8*a2;
Simulation.Parameters_IMUNoisePSD.TC_qwz=0.1*a2;
Simulation.Parameters_IMUNoisePSD.TC_qBax=0.01*a3;
Simulation.Parameters_IMUNoisePSD.TC_qBay=500*a3;
Simulation.Parameters_IMUNoisePSD.TC_qBaz=.01*a3;
Simulation.Parameters_IMUNoisePSD.TC_qBwx=5*a4;
Simulation.Parameters_IMUNoisePSD.TC_qBwy=5*a4;
Simulation.Parameters_IMUNoisePSD.TC_qBwz=.1*a4;
