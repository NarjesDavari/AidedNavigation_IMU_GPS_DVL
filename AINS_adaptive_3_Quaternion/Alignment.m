function [ Simulation ] = Alignment( Simulation , fb , g , I )

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        Simulation.Output.Alignment.RP_counter = Simulation.Output.Alignment.RP_counter + 1;

        Simulation.Output.Alignment.theta(Simulation.Output.Alignment.RP_counter)  = asin((fb(1))/g);
%         
        Simulation.Output.Alignment.phi(Simulation.Output.Alignment.RP_counter)    = -asin((fb(2))/...
                                                                (g*cos(Simulation.Output.Alignment.theta(Simulation.Output.Alignment.RP_counter))));
%         Simulation.Output.Alignment.time(Simulation.Output.Alignment.RP_counter,1) = Simulation.Input.Measurements.IMU(I,1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

