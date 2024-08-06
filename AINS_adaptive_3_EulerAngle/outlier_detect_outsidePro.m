% clear
% clc

load('Real_Measurement_testReal_LSTS_5Jun', 'Real_Measurement')
DVL(:,1)=Real_Measurement.DVL(:,2);
length_data = length(DVL(:,1));
length_Win=10;
 
% kk=1;
for j=1:length_data-length_Win %%%floor(length_data/length_Win)
%     v(:,1)=sort(DVL((j-1)*length_Win+1:j*length_Win,1));
v(:,1)=sort(DVL(j:(j-1)+length_Win,1));
    k=1;
    for q=0.25:0.25:0.75
        i(k)= floor(q*(length_Win-1)+1);
        v_q (k,j)= v(i(k),1) + ((length_Win-1)*q - i(k) + 1)*(v(i(k)+1,1)-v(i(k),1));
        k=k+1;
    end
     x_u = v_q(3,j);
         x_l = v_q (1,j);
         upper_lim=(2.5*x_u - 1.5*x_l);
         lower_lim=(2.5*x_l - 1.5*x_u);
         
   for kk=j:(j-1)+length_Win
         if kk==1
             DVL_new(1,1)=DVL(1,1);
         else
            if DVL(kk,1)>upper_lim || DVL(kk,1)<lower_lim
                DVL_new(kk,1)=mean(v_q(:,j));
            else
                 DVL_new(kk,1)=DVL(kk,1);
            end
         end
%         kk=kk+1;
    end
    
end


    
figure;plot(DVL);hold on;plot(DVL_new,'r')