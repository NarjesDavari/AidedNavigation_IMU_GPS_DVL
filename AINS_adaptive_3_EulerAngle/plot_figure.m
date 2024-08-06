%plot of different figures
function  plot_figure( Simulation,meter_3D,deg_3D,absolute_error,relative_error )
    
    %$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    ave_sample = Simulation.Init_Value.ave_sample;
    %$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    Time    = Simulation.Input.Measurements.IMU(ave_sample:end,1,1);
    Tlength = length(Simulation.Input.Measurements.IMU(ave_sample:end,1,1));        
        if meter_3D
            figure
            plot3(Simulation.Input.Measurements.Ref_Pos(ave_sample:end,3),Simulation.Input.Measurements.Ref_Pos(ave_sample:end,2),Simulation.Input.Measurements.Ref_Pos(ave_sample:end,4),'m')
            hold on
            plot3(Simulation.Output.ESKF.Pos_m(:,2),Simulation.Output.ESKF.Pos_m(:,1),Simulation.Output.ESKF.Pos_m(:,3))
            legend('Designed path in meter','Estimated path in meter');
            xlabel('Xe position(m)');
            ylabel('Xn position(m)');
            zlabel('z position(m)');
            grid on 
        end  
        %$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        if deg_3D
            figure
            plot3(Simulation.Input.User_Def_Sim.Path.P_deg(:,2),Simulation.Input.User_Def_Sim.Path.P_deg(:,1),Simulation.Input.User_Def_Sim.Path.P_deg(:,3),'m')
            hold on
            plot3(Simulation.Output.INS.X_INS(:,2)*180/pi,Simulation.Output.INS.X_INS(:,1)*180/pi,Simulation.Output.INS.X_INS(:,3));
            legend('Designed path in deg','Estimated path in deg');
            xlabel('lon position(radian)');
            ylabel('lat position(radian)');
            zlabel('alt position(m)');
            grid on
        end      
        %$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        if relative_error 
            figure
            plot(Time(1:Tlength-1),Simulation.Output.ESKF.Pos_Error.relative_error,'m')
            title('Relative Error');
            xlabel('Time(s)');
            ylabel('Relative Error');
            grid on            
        end
        %$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        if absolute_error
            figure
            plot(Time(1:Tlength),Simulation.Output.ESKF.Pos_Error.absolute_error,'m')
            title(' The distance between designed and estimated path ');
            xlabel('Time(s)');
            ylabel('absolute error(m)');   
            grid on            
        end
%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

end