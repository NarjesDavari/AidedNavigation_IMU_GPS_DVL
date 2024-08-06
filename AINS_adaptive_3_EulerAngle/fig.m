
%% waypoint
figure;plot(ans.Output.ESKF.Pos_Error.RMSE)
figure;plot(ans.Input.Measurements.Ref_Pos(:,1),ans.Input.Measurements.Ref_Pos(1:end,3))
hold on;plot(ans.Input.Measurements.IMU(:,1),ans.Output.ESKF.Pos_m(:,2),'r')
figure;plot(ans.Input.Measurements.Ref_Pos(:,1),ans.Input.Measurements.Ref_Pos(1:end,2))
hold on;plot(ans.Input.Measurements.IMU(:,1),ans.Output.ESKF.Pos_m(:,1),'r')
figure;plot(ans.Input.Measurements.Depth(:,1),ans.Input.Measurements.Depth(:,2))
hold on;plot(ans.Input.Measurements.IMU(:,1),ans.Output.ESKF.Pos_m(:,3),'r')
%% 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;plot(Simulation.Input.Measurements.Ref_Pos(1:end,3),Simulation.Input.Measurements.Ref_Pos(1:end,2))
hold on;plot(Simulation.Output.ESKF.Pos_m(:,2),Simulation.Output.ESKF.Pos_m(:,1),'r')

hold on;plot(ans.Input.Measurements.IMU(1:end,1),ans.Outpfigure;plot(ans.Input.Measurements.Depth(1:end,1),ans.Input.Measurements.Depth(1:end,2))
ut.ESKF.Pos_m(1:end,3),'r')


figure;plot(Simulation.Input.Measurements.IMU(:,1),Simulation.Output.ESKF.Pos_Error.absolute_error)

figure;plot(ans.Input.Measurements.Heading(:,1),ans.Input.Measurements.Heading(:,2))
hold on;plot(ans.Input.Measurements.IMU(:,1),ans.Output.ESKF.O_corrected(:,9)*180/pi,'r')


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

%% estimated Velocity is converted to body frame
for i=1:length(ans_113801_LargOut_reject_3.Input.Measurements.IMU(:,1))
V_body(:,i)=inv(ans_113801_LargOut_reject_3.Output.ESKF.Cbn_corrected(:,:,i))*[ans_113801_LargOut_reject_3.Output.ESKF.O_corrected(i,4);ans_113801_LargOut_reject_3.Output.ESKF.O_corrected(i,5);ans_113801_LargOut_reject_3.Output.ESKF.O_corrected(i,6)];
end
err_V=mean(sum((V_body-V_body_ref).^2,2));
V_horizontal = sqrt(V_body(:,1).^2+V_body(:,2).^2);%m/sec
%% 

for i=1:length(E3)
    if E3(i)>0.8
        E3_n(i)=0.4;
    elseif E3(i)<-0.9
        E3_n(i)=-0.3;
    else
        E3_n(i)=E3(i);
    end
end

for i=1:length(E2)
    if E2(i)<-1 && E2(i)>-1.5
        E2_n(i)=E2(i)+0.5;
    elseif E2(i)<-1.5
        E2_n(i)=E2(i)+1.5;
    else
        E2_n(i)=E2(i);
    end
end

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

%% 
 S_v(:,1)=nonzeros(ans.Output.Kalman_mtx.S_Vn(:,1));
S_v(:,2)=nonzeros(ans.Output.Kalman_mtx.S_Vn(:,2));
S_v(:,3)=nonzeros(ans.Output.Kalman_mtx.S_Vn(:,3));
dv(:,1)=nonzeros(ans.Output.Kalman_mtx.dz_Vn(:,1));
dv(:,2)=nonzeros(ans.Output.Kalman_mtx.dz_Vn(:,2));
dv(:,3)=nonzeros(ans.Output.Kalman_mtx.dz_Vn(:,3));
t=ans.Input.Measurements.DVL(:,1);
chi_square = repmat(25,length(t),1);
figure;subplot(2,1,1);plot(t,ans.Input.Measurements.DVL(:,2))
subplot(2,1,2);plot(t,sqrt(dv(:,1).^2./S_v(:,1)))

for i=1:length(t)
    nis2(i,1)=dv(i,:)*(diag(S_v(i,:)))^(-1)*dv(i,:)';
end

figure;subplot(3,1,1);plot(t,dv1(:,1).^2./S_v1(:,1));hold on; plot(t,chi_square,'r')
subplot(3,1,2);plot(t,dv1(:,2).^2./S_v1(:,2));hold on; plot(t,chi_square,'r')
subplot(3,1,3);plot(t,dv1(:,3).^2./S_v1(:,3));hold on; plot(t,chi_square,'r')

%% 


m=0.1;
coef=[.5,.7,1];
velocity=Real_Measurement.DVL(1:19700,2);
n= length(velocity);
vel2(:,1)=velocity;
vel2(:,2)=velocity;
vel2(:,3)=velocity;
for i=1:n*m
dim(i,1)=randi([1,n],1,1);
for j=1:3
    vel2(dim(i,1),j)=(1+coef(j))*velocity(dim(i,1));
end
end

% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Horizontal Velocity
VNED2=res2.Output.ESKF.O_corrected(:,4:6);
VNED3=res3.Output.ESKF.O_corrected(:,4:6);
euler2=res2.Output.ESKF.O_corrected(:,7:9);
euler3=res3.Output.ESKF.O_corrected(:,7:9);
CBN2 = InCBN_test( euler2 )
CBN3 = InCBN_test( euler3 );
for i=1:79789
V2(i,:)=(CBN2(:,:,i)*VNED2(i,:)')';
V3(i,:)=(CBN3(:,:,i)*VNED3(i,:)')';
end
for i=1:79789
Vh2(i,1)=sqrt(V2(i,1)^2+V2(i,2)^2);
Vh3(i,1)=sqrt(V3(i,1)^2+V3(i,2)^2);
end

