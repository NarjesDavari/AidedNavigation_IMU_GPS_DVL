%computation of distance of P(j+1) from P(j) in Reference(designed) Path & 
%Travelled distance on the timestep j with respect to initial point &
%distance between Point in Reference(real) path and travelled(computed) path and
%navigation error at every timestep
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ Simulation] = Navigate_Error_wayPoint( Simulation , ave_sample )
    
         %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
            Dlength            = length(Simulation.Input.Measurements.IMU);
            %Creation of space in memory for relative and absolute error
            Simulation.Output.ESKF.Pos_Error.relative_error = [];
            Simulation.Output.ESKF.Pos_Error.absolute_error = []; 
            Simulation.Output.ESKF.Pos_Error.RMSE=[];
            Simulation.Output.ESKF.Pos_Error.RMSEx=[];
            Simulation.Output.ESKF.Pos_Error.RMSEy=[];
            Simulation.Output.ESKF.Pos_Error.RMSEz=[];            
            i=1;j=1;k=1;n=1;stop=0;flag=0;
            while i<=Dlength && flag==0
                if abs(Simulation.Input.Measurements.IMU(i,1)-Simulation.Input.Measurements.GPS(j,1))<0.011 && j<=length(Simulation.Input.Measurements.GPS)
                    diff_P_Pos(k,:) =Simulation.Input.Measurements.Ref_Pos(j,2:3)-Simulation.Output.ESKF.Pos_m(i,1:2);
                     sqr_diff_P_XPos(k,:) = diff_P_Pos(k,:).^2;
                     
                    diff_P_Posx(k,:)=Simulation.Input.Measurements.Ref_Pos(j,2) - Simulation.Output.ESKF.Pos_m(i,1);
                    diff_P_Posy(k,:)=Simulation.Input.Measurements.Ref_Pos(j,3) - Simulation.Output.ESKF.Pos_m(i,2);
                    diff_P_Posz(k,:)=Simulation.Input.Measurements.Ref_Pos(j,4) - Simulation.Output.ESKF.Pos_m(i,3);

                    sqr_diff_P_XPos_x(k,:)=diff_P_Posx(k,:).^2;
                    sqr_diff_P_XPos_y(k,:)=diff_P_Posy(k,:).^2;
                    sqr_diff_P_XPos_z(k,:)=diff_P_Posz(k,:).^2;
                    j=j+1;
                    k=k+1;
                    if  j>length(Simulation.Input.Measurements.GPS)
                        stop=1;
                        j=j-1;flag=1;
                    end
                elseif Simulation.Input.Measurements.GPS(j,1)< Simulation.Input.Measurements.IMU(i,1)+0.011
                       j=j+1;
                end
                if (j>1 && k~=1  && (Simulation.Input.Measurements.GPS(j,1)-Simulation.Input.Measurements.GPS(j-1,1))>1.5) || stop==1
                    Simulation.Output.ESKF.Pos_Error.RMSE(n,1)=sqrt(mean(sum(sqr_diff_P_XPos(1:k-1,:),2)));
                    travelled_distance=cumsum(sqrt(sum([diff(Simulation.Input.Measurements.Ref_Pos(1:k-1,2)) diff(Simulation.Input.Measurements.Ref_Pos(1:k-1,3))].^2,2))); %%diff(Simulation.Input.Measurements.Ref_Pos(1:k-1,4))
                    Simulation.Output.ESKF.Pos_Error.Relative_RMSE(n,1)=Simulation.Output.ESKF.Pos_Error.RMSE(n,1)*100/travelled_distance(end,1);
                    
                    Simulation.Output.ESKF.Pos_Error.RMSEx(n,1)=sqrt(mean(sum(sqr_diff_P_XPos_x(1:k-1,:),2)));
                    travelled_distancex= cumsum(abs(diff(Simulation.Input.Measurements.Ref_Pos(1:k-1,2))));
                    Simulation.Output.ESKF.Pos_Error.Relative_RMSEx(n,1)=Simulation.Output.ESKF.Pos_Error.RMSEx(n,1)*100/ travelled_distancex(end,1);
                    
                    Simulation.Output.ESKF.Pos_Error.RMSEy(n,1)=sqrt(mean(sum(sqr_diff_P_XPos_y(1:k-1,:),2)));
                    travelled_distancey= cumsum(abs(diff(Simulation.Input.Measurements.Ref_Pos(1:k-1,3))));
                     Simulation.Output.ESKF.Pos_Error.Relative_RMSEy(n,1)=Simulation.Output.ESKF.Pos_Error.RMSEy(n,1)*100/ travelled_distancey(end,1);
                     
                    Simulation.Output.ESKF.Pos_Error.RMSEz(n,1)=sqrt(mean(sum(sqr_diff_P_XPos_z(1:k-1,:),2)));
                    travelled_distancez= cumsum(abs(diff(Simulation.Input.Measurements.Ref_Pos(1:k-1,4))));
                     Simulation.Output.ESKF.Pos_Error.Relative_RMSEz(n,1)=Simulation.Output.ESKF.Pos_Error.RMSEz(n,1)*100/ travelled_distancez(end,1);
                     
                     Simulation.Output.ESKF.Pos_Error.absolute_error=sqrt(sum(diff_P_Pos(1:k-1,:).^2,2));
                     Simulation.Output.ESKF.Pos_Error.absolute_error_end(n,1)=Simulation.Output.ESKF.Pos_Error.absolute_error(end,1);
                    k=1;
                    n=n+1;
                    stop=0;
                end   
                    i=i+1;
            end      
end