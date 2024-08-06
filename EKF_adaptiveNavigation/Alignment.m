function [ Simulation ] = Alignment( Simulation , I , fb , gg )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        Simulation.Output.Alignment.RP_counter = Simulation.Output.Alignment.RP_counter + 1;
    
        Simulation.Output.Alignment.theta(Simulation.Output.Alignment.RP_counter,1) = asin((fb(1))/gg);
        
        Simulation.Output.Alignment.phi(Simulation.Output.Alignment.RP_counter,1)   = -asin((fb(2))/...
                                                                (gg*cos(Simulation.Output.Alignment.theta(Simulation.Output.Alignment.RP_counter,1))));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
