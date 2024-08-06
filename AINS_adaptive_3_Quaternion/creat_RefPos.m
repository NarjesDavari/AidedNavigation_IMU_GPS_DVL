
function [Simulation]= creat_RefPos(Simulation,t_imu)
flag=0;
t_im=t_imu;
GPS(:,1)= ((round(Simulation.Input.Measurements.GPS(:,1)*1000))/1000);
GPS(:,2:4)=Simulation.Input.Measurements.GPS(:,2:4);
gps_m=conversion_geo2tan(GPS);
Simulation.Input.Measurements.Ref_Pos=zeros(length(t_im),4); 
gps_m2(:,1)=[t_im(1,1);gps_m(:,1)];
gps_m2(:,2)=[gps_m(1,2);gps_m(:,2)];
gps_m2(:,3)=[gps_m(1,3);gps_m(:,3)];

if gps_m2(end,1)< t_im(end,1)
    ref1(:,1)=[gps_m2(:,1);t_im(end,1)];
    ref1(:,2)=[gps_m2(:,2);gps_m2(end,2)];
    ref1(:,3)=[gps_m2(:,3);gps_m2(end,3)];
%     ref1(:,4)=[gps_m(:,4);gps_m(end,4)];
    t_imu=t_im;
    
else
    t_imu=[t_im;gps_m(end,1)];
    ref1(:,1)=gps_m2(:,1);
    ref1(:,2)=gps_m2(:,2);
    ref1(:,3)=gps_m2(:,3);
%     ref1(:,4)=gps_m(:,4);
flag=1;
end

Ref(:,1)=t_imu;
Ref(:,2)=interp1(ref1(:,1),ref1(:,2),t_imu);
Ref(:,3)=interp1(ref1(:,1),ref1(:,3),t_imu);
% Ref(:,4)=interp1(ref1(:,1),ref1(:,4),t_imu);
if flag==1
    Simulation.Input.Measurements.Ref_Pos(:,1)=Ref(1:end-1,1);
    Simulation.Input.Measurements.Ref_Pos(:,2)=Ref(1:end-1,2);
    Simulation.Input.Measurements.Ref_Pos(:,3)=Ref(1:end-1,3);
    Simulation.Input.Measurements.Ref_Pos(:,4)=median(Simulation.Input.Measurements.Depth(:,2));
else
     Simulation.Input.Measurements.Ref_Pos(:,1)=Ref(1:end,1);
    Simulation.Input.Measurements.Ref_Pos(:,2)=Ref(1:end,2);
    Simulation.Input.Measurements.Ref_Pos(:,3)=Ref(1:end,3);
    Simulation.Input.Measurements.Ref_Pos(:,4)=median(Simulation.Input.Measurements.Depth(:,2));
end