function [Simulation,t1,t2]=Data_Lose_DVL(Simulation,N,time_outage,Start_Time_out,Dlength,ave_sample)
timeinterval_imu=Simulation.Input.Measurements.IMU(2,1)-Simulation.Input.Measurements.IMU(1,1);
timeinterval_dvl=Simulation.Input.Measurements.DVL(2,1)-Simulation.Input.Measurements.DVL(1,1);

interval_lose=round(time_outage/timeinterval_dvl);
% t1=zeros(1,N);
% t2=zeros(1,N);
init=round(Start_Time_out*60/timeinterval_dvl);   
dvl1=Simulation.Input.Measurements.DVL;

Dlength_dvl=length(Simulation.Input.Measurements.DVL(:,1));
% dvl=zeros(Dlength,4);
L_G=Dlength_dvl;
distance_data=round((L_G-init-N*interval_lose-N)/N);
dvl(1:init,:)=dvl1(1:init,:);
for i=1:N
    dvl(init+1+(i-1)*distance_data:init+i*distance_data,:)=dvl1(init+i*interval_lose+(i-1)*distance_data+1:init+i*(interval_lose+distance_data),:);
    t11(i)=dvl1(init+(i-1)*(interval_lose+distance_data)+1,1);
    t22(i)=dvl1(init+i*(interval_lose)+(i-1)*distance_data,1);
     t1_=((round(t11(i)))/timeinterval_imu)+1; %%time step in IMU 
    t2_=((round(t22(i)))/timeinterval_imu)+1;   %%time step in IMU
    if t2_< Dlength  %%1.923e5 for car
        t1(i)=t1_;
        t2(i)=t2_;
    end
end




Simulation.Input.Measurements.DVL   = dvl;
