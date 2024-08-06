%computation of distance of P(j+1) from P(j) in Reference(designed) Path & 
%Travelled distance on the timestep j with respect to initial point &
%distance between Point in Reference(real) path and travelled(computed) path and
%navigation error at every timestep
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ Simulation] = Navigate_Error( Simulation , ave_sample )
    
         %@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
            Dlength            = length(Simulation.Input.Measurements.IMU);
            %Creation of space in memory for relative and absolute error
            Simulation.Output.ESKF.Pos_Error.relative_error = zeros(Dlength - ave_sample,1);
            Simulation.Output.ESKF.Pos_Error.absolute_error = zeros(Dlength - ave_sample + 1,1); 
            Simulation.Output.ESKF.Pos_Error.RMSE=[];
            Simulation.Output.ESKF.Pos_Error.RMSEx=[];
            Simulation.Output.ESKF.Pos_Error.RMSEy=[];
            Simulation.Output.ESKF.Pos_Error.RMSEz=[];            
            
            %inserting of travelled time and distance in memory
            Simulation.Output.ESKF.Pos_Error.travelled_time     = Simulation.Input.Measurements.IMU(end,1) - Simulation.Input.Measurements.IMU(ave_sample,1);%Sec  
            Simulation.Output.ESKF.Pos_Error.travelled_distance = Simulation.Input.Path.s_i(end); 
            
            Simulation.Output.ESKF.Pos_Error.travelled_distancex=Simulation.Input.Path.sx_i(end) ;
            Simulation.Output.ESKF.Pos_Error.travelled_distancey=Simulation.Input.Path.sy_i(end) ;
            Simulation.Output.ESKF.Pos_Error.travelled_distancez=Simulation.Input.Path.sz_i(end) ;
            %computation of the difference of Points in designed(Reference) path and
            %estimated(computed) path  at every timestep
                diff_P_Pos      =Simulation.Input.Measurements.Ref_Pos(ave_sample:end,2:4)-Simulation.Output.ESKF.Pos_m(:,:);%(:,3) 
                
                sqr_diff_P_XPos =diff_P_Pos.^2;%(:,3)
                
                Simulation.Output.ESKF.Pos_Error.RMSE=sqrt(mean(sum(sqr_diff_P_XPos,2)));
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                diff_P_Posx=Simulation.Input.Measurements.Ref_Pos(ave_sample:end,2) - Simulation.Output.ESKF.Pos_m(:,1);
                diff_P_Posy=Simulation.Input.Measurements.Ref_Pos(ave_sample:end,3) - Simulation.Output.ESKF.Pos_m(:,2);
                diff_P_Posz=Simulation.Input.Measurements.Ref_Pos(ave_sample:end,4) - Simulation.Output.ESKF.Pos_m(:,3);
                
                sqr_diff_P_XPos_x=diff_P_Posx.^2;
                sqr_diff_P_XPos_y=diff_P_Posy.^2;
                sqr_diff_P_XPos_z=diff_P_Posz.^2;
                
                Simulation.Output.ESKF.Pos_Error.RMSEx=sqrt(mean(sum(sqr_diff_P_XPos_x,2)));
                Simulation.Output.ESKF.Pos_Error.RMSEy=sqrt(mean(sum(sqr_diff_P_XPos_y,2)));
                Simulation.Output.ESKF.Pos_Error.RMSEz=sqrt(mean(sum(sqr_diff_P_XPos_z,2)));
                                
                Simulation.Output.ESKF.Pos_Error.Relative_RMSEx=Simulation.Output.ESKF.Pos_Error.RMSEx*100/...
                                                                                 Simulation.Output.ESKF.Pos_Error.travelled_distancex;
                Simulation.Output.ESKF.Pos_Error.Relative_RMSEy=Simulation.Output.ESKF.Pos_Error.RMSEy*100/...
                                                                                 Simulation.Output.ESKF.Pos_Error.travelled_distancey;
                Simulation.Output.ESKF.Pos_Error.Relative_RMSEz=Simulation.Output.ESKF.Pos_Error.RMSEz*100/...
                                                                                 Simulation.Output.ESKF.Pos_Error.travelled_distancez;                                                                            
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%               
                %absolute error between Points in designed(real) path and estimated(computed) path at every timestep
                absolute_error=sqrt(sum(sqr_diff_P_XPos,2));
                
                %computation of navigation error at every timestep
                relative_error=(absolute_error(1:end-1)./Simulation.Input.Path.s_i)*100;                 
                Simulation.Output.ESKF.Pos_Error.relative_error=relative_error;
                Simulation.Output.ESKF.Pos_Error.absolute_error=absolute_error;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                           
                Simulation.Output.ESKF.Pos_Error.Relative_RMSE=Simulation.Output.ESKF.Pos_Error.RMSE*100/Simulation.Output.ESKF.Pos_Error.travelled_distance;
            
                Simulation.Output.ESKF.Pos_Error.absolute_error_end=Simulation.Output.ESKF.Pos_Error.absolute_error(end,1);
                Simulation.Output.ESKF.Pos_Error.relative_error_end=Simulation.Output.ESKF.Pos_Error.relative_error(end,1);
          
end