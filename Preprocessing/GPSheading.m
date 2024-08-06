function [ Simulation ] = GPSheading( Real_Measurement )

    Time=Real_Measurement.GPS_Smooth_meter(:,1);
    
    Dlength=length(Time);
    
%     dXn=diff(Simulation.kalman_filter.xn_s);
%     dXe=diff(Simulation.kalman_filter.xe_s); 
    dXn=diff(Real_Measurement.GPS_Smooth_meter(:,2));
    dXe=diff(Real_Measurement.GPS_Smooth_meter(:,3));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     Simulation.GPSheading1(:,1)=Time(1:end-1);
%     for i=1:Dlength-1
%         H=atan2(dXe(i),dXn(i))*180/pi;
%         heading(i,2)=H;       
%     end
%     Simulation.GPSheading1(:,2)=heading(:,2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Simulation.GPSheading.Heading1(:,1)=Time(1:end-1);     
    for i=1:Dlength-1
        H=atan(dXe(i)/dXn(i))*180/pi;
        if dXn(i)>0
            heading(i,2)=H;
        end
        if (dXn(i)<0) 
              if  (H<0)
                    heading(i,2)=H+180;
              end
        end    
        if (dXn(i)<0)
               if (H>0)
                    heading(i,2)=H-180;
               end
        end        
    end
    Simulation.GPSheading.Heading1(:,2)=heading(:,2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
end
