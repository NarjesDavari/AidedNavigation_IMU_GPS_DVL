function [Simulation,rela_RMSE,RMSE_i,RMSE_outage,absolute_error_outage,absolute_error_end] = CalculError (Simulation,t1,t2,ave_sample  )
Dlength = length(Simulation.Input.Measurements.IMU);
         
            travelled_distance = Simulation.Input.Path.s_i(t2-ave_sample)-Simulation.Input.Path.s_i(t1-ave_sample); 
            travelled_distancex=Simulation.Input.Path.sx_i(t2-ave_sample)-Simulation.Input.Path.sx_i(t1-ave_sample);
            travelled_distancey=Simulation.Input.Path.sy_i(t2-ave_sample)-Simulation.Input.Path.sy_i(t1-ave_sample);
            travelled_distancez=Simulation.Input.Path.sz_i(t2-ave_sample)-Simulation.Input.Path.sz_i(t1-ave_sample);
            travelled_time = t2-t1 ;
            %computation of the difference of Points in designed(Reference) path and
            %estimated(computed) path  at every timestep
            
%                 diff_P_Pos (i,:)=Simulation.Input.Measurements.Ref_Pos(i,2:4)-Simulation.Output.ESKF.Pos_m(i,:);%(:,3) 
                diff_P_Posx=Simulation.Input.Measurements.Ref_Pos(t1:t2,2)-Simulation.Output.ESKF.Pos_m(t1-ave_sample:t2-ave_sample,1);
                diff_P_Posy=Simulation.Input.Measurements.Ref_Pos(t1:t2,3)-Simulation.Output.ESKF.Pos_m(t1-ave_sample:t2-ave_sample,2);
                diff_P_Posz=Simulation.Input.Measurements.Ref_Pos(t1:t2,4)-Simulation.Output.ESKF.Pos_m(t1-ave_sample:t2-ave_sample,3);
  
                
%                 sqr_diff_P_XPos =diff_P_Pos.^2;%(:,3)
                
%                 Simulation.Output.ESKF.Pos_Error.RMSE_i(i,:)=sqrt(mean(sum(sqr_diff_P_XPos,2)));
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                sqr_diff_P_XPos_x=diff_P_Posx.^2;
                sqr_diff_P_XPos_y=diff_P_Posy.^2;
                sqr_diff_P_XPos_z=diff_P_Posz.^2;
                
                RMSEx_i=sqrt(mean(sum(sqr_diff_P_XPos_x,2)));
                RMSEy_i=sqrt(mean(sum(sqr_diff_P_XPos_y,2)));
                RMSEz_i=sqrt(mean(sum(sqr_diff_P_XPos_z,2)));
                RMSE_i=[RMSEx_i RMSEy_i RMSEz_i];
                
%                 Simulation.Output.ESKF.Pos_Error.RMSEx_ave=mean(Simulation.Output.ESKF.Pos_Error.RMSEx_i);
%                 Simulation.Output.ESKF.Pos_Error.RMSEy_ave=mean(Simulation.Output.ESKF.Pos_Error.RMSEy_i);
%                 Simulation.Output.ESKF.Pos_Error.RMSEz_ave=mean(Simulation.Output.ESKF.Pos_Error.RMSEz_i);              
                Relative_RMSEx_ave=RMSEx_i*100/travelled_distancex;
                Relative_RMSEy_ave=RMSEy_i*100/travelled_distancey;
                Relative_RMSEz_ave=RMSEz_i*100/travelled_distancez; 
                rela_RMSE=[Relative_RMSEx_ave Relative_RMSEy_ave  Relative_RMSEz_ave];
                                                                         
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                diff_P_Pos =Simulation.Input.Measurements.Ref_Pos(t1:t2,2:4)-Simulation.Output.ESKF.Pos_m(t1-ave_sample:t2-ave_sample,:);     
                RMSE_outage=sqrt(mean(sum(diff_P_Pos.^2,2)));
                

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %absolute error between Points in designed(real) path and estimated(computed) path at every timestep
                 absolute_error_outage=sqrt(sum(diff_P_Pos.^2,2));
                 absolute_error_end=absolute_error_outage(end,1);
                
                %computation of navigation error at every timestep
%                 relative_error_i=(absolute_error_i(1:Dlength-1)./Simulation.Input.Path.s_i)*100;                 

                
                