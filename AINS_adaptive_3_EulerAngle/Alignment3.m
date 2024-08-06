function [ Simulation ] = Alignment3( Simulation , fb , g , I , muu , ave_sample )
        frp=20;
        Srp=100/frp;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if Simulation.Output.Alignment.norm.Accel_norm(I - ave_sample + 1) <  muu
            Simulation.Output.Alignment.RP_counter = Simulation.Output.Alignment.RP_counter + 1;
            
            Simulation.Output.Alignment.g(I - ave_sample + 1 ,1)  = g;
            Simulation.Output.Alignment.fx(I - ave_sample + 1 ,1) = fb(1);
            Simulation.Output.Alignment.fy(I - ave_sample + 1 ,1) = fb(2);            
    
        end
%         Simulation.Output.Alignment.time(Simulation.Output.Alignment.RP_counter,1) = Simulation.Input.Measurements.IMU(I,1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if rem(I - ave_sample + 1,Srp)==0
            Simulation.Output.Alignment.RP_counter2 = Simulation.Output.Alignment.RP_counter2 + 1;
            J = Simulation.Output.Alignment.RP_counter2;
            if ~isempty(nonzeros(Simulation.Output.Alignment.fx((J-1)*Srp+1:J*Srp,1)))    
                Simulation.Output.Alignment.RP_active = 1;
                
                Simulation.Output.Alignment.ave_RP_counter = Simulation.Output.Alignment.ave_RP_counter + 1;
                
                Simulation.Output.Alignment.ave_fx(Simulation.Output.Alignment.ave_RP_counter,1) = ...
                                sum(nonzeros(Simulation.Output.Alignment.fx((J-1)*Srp+1:J*Srp,1)))/length(nonzeros(Simulation.Output.Alignment.fx((J-1)*Srp+1:J*Srp,1)));
                
                Simulation.Output.Alignment.ave_fy(Simulation.Output.Alignment.ave_RP_counter,1) = ...
                                sum(nonzeros(Simulation.Output.Alignment.fy((J-1)*Srp+1:J*Srp,1)))/length(nonzeros(Simulation.Output.Alignment.fy((J-1)*Srp+1:J*Srp,1)));

                Simulation.Output.Alignment.ave_g(Simulation.Output.Alignment.ave_RP_counter,1) = ...
                                sum(nonzeros(Simulation.Output.Alignment.g((J-1)*Srp+1:J*Srp,1)))/length(nonzeros(Simulation.Output.Alignment.g((J-1)*Srp+1:J*Srp,1))); 
                            
                Simulation.Output.Alignment.theta(Simulation.Output.Alignment.ave_RP_counter,1) = ...
                                                               asin(Simulation.Output.Alignment.ave_fx(Simulation.Output.Alignment.ave_RP_counter,1)/...
                                                               Simulation.Output.Alignment.ave_g(Simulation.Output.Alignment.ave_RP_counter,1));
            
                Simulation.Output.Alignment.phi(Simulation.Output.Alignment.ave_RP_counter,1)   = ...
                                                              -asin(Simulation.Output.Alignment.ave_fy(Simulation.Output.Alignment.ave_RP_counter,1)/...
                                                              (Simulation.Output.Alignment.ave_g(Simulation.Output.Alignment.ave_RP_counter,1) * ...
                                                               cos(Simulation.Output.Alignment.theta(Simulation.Output.Alignment.ave_RP_counter,1))));                            
            end
        end
end

