function [ Simulation ] = State_Correction( Simulation , I , ave_sample )
        
    global Updt_Cntr;
    global Cbn_det;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Simulation.Output.ESKF.O_corrected(I - ave_sample + 1,1:6) = Simulation.Output.INS.X_INS(I - ave_sample + 1,1:6)+ ...
                                                                     Simulation.Output.ESKF.dX(I - ave_sample + 1,1:6);  
    
            Simulation.Output.INS.X_INS(I - ave_sample + 1,1:6)   = Simulation.Output.ESKF.O_corrected(I - ave_sample + 1,1:6);  
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Limitation
            if Simulation.Output.INS.X_INS(I - ave_sample + 1,3) > Simulation.Limit_Param.Z_max
                Simulation.Output.INS.X_INS(I - ave_sample + 1,3) = Simulation.Limit_Param.Z_max;
            end
            if Simulation.Output.INS.X_INS(I - ave_sample + 1,3) < Simulation.Limit_Param.Z_min
                Simulation.Output.INS.X_INS(I - ave_sample + 1,3) = Simulation.Limit_Param.Z_min;
            end
            if Simulation.Output.INS.X_INS(I - ave_sample + 1,4) > Simulation.Limit_Param.Vt
                Simulation.Output.INS.X_INS(I - ave_sample + 1,4) = Simulation.Limit_Param.Vt;
            end
            if Simulation.Output.INS.X_INS(I - ave_sample + 1,4) < -Simulation.Limit_Param.Vt
                Simulation.Output.INS.X_INS(I - ave_sample + 1,4) = -Simulation.Limit_Param.Vt;
            end    
            if Simulation.Output.INS.X_INS(I - ave_sample + 1,5) > Simulation.Limit_Param.Vt
                Simulation.Output.INS.X_INS(I - ave_sample + 1,5) = Simulation.Limit_Param.Vt;
            end
            if Simulation.Output.INS.X_INS(I - ave_sample + 1,5) < -Simulation.Limit_Param.Vt
                Simulation.Output.INS.X_INS(I - ave_sample + 1,5) = -Simulation.Limit_Param.Vt;
            end    
            if Simulation.Output.INS.X_INS(I - ave_sample + 1,6) > Simulation.Limit_Param.Vt
                Simulation.Output.INS.X_INS(I - ave_sample + 1,6) = Simulation.Limit_Param.Vt;
            end
            if Simulation.Output.INS.X_INS(I - ave_sample + 1,6) < -Simulation.Limit_Param.Vt
                Simulation.Output.INS.X_INS(I - ave_sample + 1,6) = -Simulation.Limit_Param.Vt;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            dq = Simulation.Output.ESKF.dX(I - ave_sample + 1,7:9);
            q_= Simulation.Output.INS.X_INS(I - ave_sample ,7:10); 
            E_q =[q_(4), -q_(3), q_(2)
                 q_(3),  q_(4), -q_(1)
                 -q_(2),  q_(1),  q_(4)
                 -q_(1),  -q_(2), -q_(3)];
            q_corr = (q_' +0.5*E_q *dq')';    
            Simulation.Output.ESKF.O_corrected(I - ave_sample + 1,7:10) = q_corr;
            Simulation.Output.INS.X_INS(I - ave_sample + 1,7:10)   = Simulation.Output.ESKF.O_corrected(I - ave_sample + 1,7:10);
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
            Simulation.Output.ESKF.Cbn_corrected(:,:,I - ave_sample + 1) = InCBN_quater(q_corr);
            if Updt_Cntr > 0
                Cbn_det(Updt_Cntr,1)= det(Simulation.Output.ESKF.Cbn_corrected(:,:,I - ave_sample + 1));
            end
            Simulation.Output.ESKF.Cbn_corrected(:,:,I - ave_sample + 1) = ...
            Simulation.Output.ESKF.Cbn_corrected(:,:,I - ave_sample + 1) / det (Simulation.Output.ESKF.Cbn_corrected(:,:,I - ave_sample + 1));
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            q_selfConsistency=q_corr(1)^2+q_corr(2)^2+q_corr(3)^2+q_corr(4)^2;
            if q_selfConsistency ==1
                selfConsistency=1;
            else
                selfConsistency=0;
            end
%             Limitation
            if Simulation.Output.ESKF.Cbn_corrected(3,1,I - ave_sample + 1) > Simulation.Limit_Param.Cbn_t
               Simulation.Output.ESKF.Cbn_corrected(3,1,I - ave_sample + 1) = Simulation.Limit_Param.Cbn_t; 
            end
            if Simulation.Output.ESKF.Cbn_corrected(3,1,I - ave_sample + 1) < -Simulation.Limit_Param.Cbn_t
               Simulation.Output.ESKF.Cbn_corrected(3,1,I - ave_sample + 1) = -Simulation.Limit_Param.Cbn_t; 
            end            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            phi   = atan2(2*(-q_corr(2)*q_corr(3)+q_corr(4)*q_corr(1)),1-2*((q_corr(1))^2+(q_corr(2))^2));    
            Stheta = 2*(q_corr(1)*q_corr(3)+q_corr(4)*q_corr(2));
            psi   = atan2(2*(-q_corr(1)*q_corr(2)+ q_corr(4)*q_corr(3)),1-2*((q_corr(2))^2+(q_corr(3))^2));        
            
            Simulation.Output.INS.X_INS_BEuler(I - ave_sample + 1,1:6)      = Simulation.Output.ESKF.O_corrected(I - ave_sample + 1,1:6);
             Simulation.Output.INS.X_INS_BEuler(I - ave_sample + 1,7:9) = [phi,asin(Stheta),psi];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            phi2   = atan2(Simulation.Output.ESKF.Cbn_corrected(3,2,I - ave_sample + 1),Simulation.Output.ESKF.Cbn_corrected(3,3,I - ave_sample + 1));    
            theta2 = -atan(Simulation.Output.ESKF.Cbn_corrected(3,1,I - ave_sample + 1)/sqrt(1-Simulation.Output.ESKF.Cbn_corrected(3,1,I - ave_sample + 1)^2));
            psi2   = atan2(Simulation.Output.ESKF.Cbn_corrected(2,1,I - ave_sample + 1),Simulation.Output.ESKF.Cbn_corrected(1,1,I - ave_sample + 1));         

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Roll Limitation
%             if Simulation.Output.INS.X_INS(I - ave_sample + 1,7) > Simulation.Limit_Param.Roll_t * pi/180 
%                 Simulation.Output.INS.X_INS(I - ave_sample + 1,7) = Simulation.Limit_Param.Roll_t * pi/180;
%             end
%             if Simulation.Output.INS.X_INS(I - ave_sample + 1,7) < -Simulation.Limit_Param.Roll_t * pi/180 
%                 Simulation.Output.INS.X_INS(I - ave_sample + 1,7) = - Simulation.Limit_Param.Roll_t * pi/180;
%             end             
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
              Simulation.Output.ESKF.O_corrected(I - ave_sample + 1,11:16) = Simulation.Output.INS.X_INS(I - ave_sample + 1,11:16)+ ...
                                                                 Simulation.Output.ESKF.dX(I - ave_sample + 1,10:15);
            Simulation.Output.INS.X_INS(I - ave_sample + 1,11:16)   = Simulation.Output.ESKF.O_corrected(I - ave_sample + 1,11:16); 
            
            Simulation.Output.INS.X_INS_BEuler(I - ave_sample + 1,10:15)  = Simulation.Output.INS.X_INS(I - ave_sample + 1,11:16);
end

