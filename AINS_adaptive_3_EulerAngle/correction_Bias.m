function [fb,fb_,Wib_b,Wib_b_]=correction_Bias(Simulation,f,f_,w,w_,I,ave_sample)            

            fb(1)  = f(1) + Simulation.Output.INS.X_INS(I-ave_sample,10);
            fb(2)  = f(2)  + Simulation.Output.INS.X_INS(I-ave_sample,11);
            fb(3)  = f(3)  + Simulation.Output.INS.X_INS(I-ave_sample,12);
            
            fb_(1)  = f_(1)  + Simulation.Output.INS.X_INS(I-ave_sample,10);
            fb_(2)  = f_(2)  + Simulation.Output.INS.X_INS(I-ave_sample,11);
            fb_(3)  = f_(3)  + Simulation.Output.INS.X_INS(I-ave_sample,12);
            

            Wib_b(1)  = w(1) + Simulation.Output.INS.X_INS(I-ave_sample,13);   
            Wib_b(2)  = w(2) + Simulation.Output.INS.X_INS(I-ave_sample,14);
            Wib_b(3)  = w(3) + Simulation.Output.INS.X_INS(I-ave_sample,15);
            
            Wib_b_(1)  = w_(1) + Simulation.Output.INS.X_INS(I-ave_sample,13);   
            Wib_b_(2)  = w_(2) + Simulation.Output.INS.X_INS(I-ave_sample,14);
            Wib_b_(3)  = w_(3) + Simulation.Output.INS.X_INS(I-ave_sample,15);
end