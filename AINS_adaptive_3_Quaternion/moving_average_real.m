function   y_denois =moving_average_real(y,bias,ave_sample,I,coeff_final,numberOfData)
   
         MA(4,:)=(y(4,:)+y(3,:)+y(2,:)+y(1,:))/4;
            s(4,:)=sqrt(((y(4,:)-MA(4,:)).^2+(y(3,:)-MA(4,:)).^2+(y(2,:)-MA(4,:)).^2+(y(1,:)-MA(4,:)).^2)/(4-1));
                
        for i=1:numberOfData
            if -coeff_final*s(4,i)<y(4,i)<coeff_final*s(4,i)
                y_denois(1,i)=y(4,i);
            else
                y_denois(1,i)=MA(4,i);
            end
        end