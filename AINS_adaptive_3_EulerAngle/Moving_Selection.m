function [ Simulation , include_GPS , include_depthmeter , include_dvl , include_heading , include_accelrollpitch ] = Moving_Selection( Simulation )

include_GPS              = Simulation.Auxiliary_Snsr_Nav.include_GPS;
include_depthmeter       = Simulation.Auxiliary_Snsr_Nav.include_depthmeter;
include_dvl              = Simulation.Auxiliary_Snsr_Nav.include_dvl;
include_accelrollpitch   = Simulation.Auxiliary_Snsr_Nav.include_accelrollpitch;
include_heading          = Simulation.Auxiliary_Snsr_Nav.include_heading;

Simulation.Output.Auxiliary_Snsr.include_GPS            = include_GPS;
Simulation.Output.Auxiliary_Snsr.include_depthmeter     = include_depthmeter;
Simulation.Output.Auxiliary_Snsr.include_dvl            = include_dvl;
Simulation.Output.Auxiliary_Snsr.include_accelrollpitch = include_accelrollpitch;
Simulation.Output.Auxiliary_Snsr.include_heading        = include_heading;
end

