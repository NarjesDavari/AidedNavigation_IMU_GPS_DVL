function [ Simulation ] = Alignment2( Simulation , fb , g , I , muu , ave_sample )
        frp=5;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if Simulation.Output.Alignment.norm.Accel_norm(I - ave_sample + 1) <  muu
            Simulation.Output.Alignment.RP_counter = Simulation.Output.Alignment.RP_counter + 1;
    
            Simulation.Output.Alignment.theta(I - ave_sample + 1,1)  = asin((fb(1))/g);
            
            Simulation.Output.Alignment.phi(I - ave_sample + 1,1)    = -asin((fb(2))/...
                                                                    (g*cos(Simulation.Output.Alignment.theta(I - ave_sample + 1))));
        end
%         Simulation.Output.Alignment.time(Simulation.Output.Alignment.RP_counter,1) = Simulation.Input.Measurements.IMU(I,1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if rem(I - ave_sample + 1,frp)==0
            Simulation.Output.Alignment.RP_counter2 = Simulation.Output.Alignment.RP_counter2 + 1;
            J = Simulation.Output.Alignment.RP_counter2;
            if ~isempty(nonzeros(Simulation.Output.Alignment.phi((J-1)*frp+1:J*frp,1)))    
                Simulation.Output.Alignment.RP_active = 1;
                
                Simulation.Output.Alignment.ave_RP_counter = Simulation.Output.Alignment.ave_RP_counter + 1;
                
                Simulation.Output.Alignment.ave_phi(Simulation.Output.Alignment.ave_RP_counter,1) = ...
                                sum(nonzeros(Simulation.Output.Alignment.phi((J-1)*frp+1:J*frp,1)))/length(nonzeros(Simulation.Output.Alignment.phi((J-1)*frp+1:J*frp,1)));
                
                Simulation.Output.Alignment.ave_theta(Simulation.Output.Alignment.ave_RP_counter,1) = ...
                                sum(nonzeros(Simulation.Output.Alignment.theta((J-1)*frp+1:J*frp,1)))/length(nonzeros(Simulation.Output.Alignment.theta((J-1)*frp+1:J*frp,1)));
            end
        end
end

