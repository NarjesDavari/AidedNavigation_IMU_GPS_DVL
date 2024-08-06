clear
clc
tic
load('Rea_Measurement_t2_22th_5_decSamplerate', 'Real_Measurement')
n=4;
k=1;
include_MA=0;
Real_Measurement.IMU=Real_Measurement.IMU_raw;

for a=0.1:0.5:3;
    for j=2:7
        y=Real_Measurement.IMU_raw(:,j);
        N=length(y);
        MA=zeros(N-n,1);
        s=zeros(N-n,1);
        for i=n:N
            MA(i)=(y(i-3)+y(i-2)+y(i-1)+y(i))/n;
        end
        for i=n:N
        s(i,1)=sqrt(((y(i)-MA(i))^2+(y(i-1)-MA(i))^2+(y(i-2)-MA(i))^2+(y(i-3)-MA(i))^2)/(n-1));
        end
        for i=n:N
            if -a*s<y(i)<a*s
                y(i)=y(i);
            else
                y(i)=MA(i);
            end
        end
        Real_Measurement.IMU(:,j)=y;
    end
save('Rea_Measurement_t2_22th_5_decSamplerate','Real_Measurement')
load ('Parameter_stationary_sectionMA_test2_22th_4','Simulation')
[ Simulation ] = Run_AINS(Simulation,Real_Measurement,s,include_MA );
RMSE(k)=Simulation.Output.ESKF.Pos_Error.Relative_RMSE;
k=k+1;
end

include_MA=1;
index_final=find(RMSE==min(RMSE));
s_final=a(index_final);
 load('Real_Measurement_t2_22th_1', 'Real_Measurement')
load('Parameter_stationary_vessel_test2_22th_4', 'Simulation') 
[ Simulation ] = Run_AINS(Simulation,Real_Measurement,s_final,include_MA);

toc
