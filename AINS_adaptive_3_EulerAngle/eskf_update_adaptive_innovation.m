                                                                                                    %KF_UPDATE  Kalman Filter update step
%Reference: Strapdown inertial navigation system, chapter 13 page 406 
function [Simulation,dX,P] = eskf_update_adaptive_innovation(Simulation,dX, P_minus,dz,H,I,N,C,Selection_Param)

      update_counter=Simulation.Output.Kalman_mtx.update_counter;
      Simulation.Output.Kalman_mtx.update_adaptive_counter=Simulation.Output.Kalman_mtx.update_adaptive_counter+1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %         alpha = trace(H*C/N*H'-R1)/trace(H*P_minus*H');
% %          if ~isPositiveDefinite_Rmatrix(D,1)
% %              display('Given Matrix is NOT positive definite');
% %          end

 %[Simulation]=R_adaptive(Simulation,D,N,update_counter);
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%    whitening1   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [Simulation,flag_White] = whiteness_test(Simulation,N);
% if flag_White==0
%    [T,C_white] = white(C,I);
% else
    C_white=C;
% end
% if ~isPositiveDefinite_Rmatrix(C_white,1)
%     display('C_white Matrix is NOT positive definite');
% end

         R_white = H*C_white*H' /N + H*P_minus*H';
%          x=diag(R_white);
%          for i=1:size(x,1)
%              if x(i)<0
%                 diag(R_white)=0;
%              end
%          end
         S_white = R_white + H*P_minus*H';
         [u,s,v]=svd(S_white);
         S_white=u*sqrt(s)*u';
         K_white = P_minus*H' / diag(diag(S_white));
% %       P = P_minus - K_white*S_white*K_white';              %Updated state covariance
         P = P_minus - K_white * H * P_minus;
         P = (P+P')/2;
        dX = dX + (K_white * (dz - H*dX));   %Updated state mean 
   [Simulation]=R_adaptive(Simulation,R_white,N,update_counter,I,Selection_Param);

% Simulation.Output.Kalman_mtx.C_white44(Simulation.Output.Kalman_mtx.update_counter,1)=C_white(4,4);
% Simulation.Output.Kalman_mtx.C44(Simulation.Output.Kalman_mtx.update_counter,1)=C(4,4);    

Q = K_white*H*C_white/N*H'*K_white';
% Simulation.Output.Kalman_mtx.Q_adap(:,:,I) = Q;
% Simulation.Output.Kalman_mtx.Q=Q;
%%%%%%%%%%%%%%%%%%%%%%%%  whitening2  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%  [Simulation,R] = Whitening2(Simulation,N,X_hat_minus,P_minus);     
%         S = R - H*P_minus*H';
%         K = P_minus*H'/S;                 %Computed Kalman gain
%         P = P_minus - K*S*K';             %Updated state covariance
%         P = (P+P')/2;
% %         [Up,Dp] = udu(P);
% %         P = Dp;
%         X =  X_hat_minus + (K * (Y-H*X_hat_minus'))';   %Updated state mean 
% %       Q = K*C/N*K';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% refrence: paper(Autonomous Underwater Vehicle Navigation Using an Adaptive Kalman Filter for Sensor Fusion)

% [U,D]= udu(P_minus);
% S_factor = H *U*D*U' * H' + R_conventional;
% F = D*U'*H';
% G = U*F;
% K_k= G*S_factor^-1;
% X = X_hat_minus + (K_k * (Y - H*X_hat_minus'))';   %Updated state mean
% P = U*(D-F*S_factor^-1*F');

% %        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%whitening2  
%         [T,Q] = white(C);
%         C_white = T^-1 * C * T'^-1;
%         R_white = C_white /N-H*P_minus*H';
%         S_white = R_white + H*P_minus*H';
%         K_white = P_minus*H' / diag(diag(S_white),0);
% %       P = P_minus - K_white*S_white*K_white';              %Updated state covariance
%         P = P_minus - K_white * H * P_minus;
%         X = X_hat_minus + (K_white * (Y - H*X_hat_minus'))';   %Updated state mean 

% %  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%          R = C/N-H*(P_minus)*H';
%          R = (R+R')/2;
%          if ~isPositiveDefinite_Rmatrix(R,1)
%              display('Given Matrix is NOT positive definite');
% %              [X , P] = ekf_update1(Simulation,X_conventional , P_conventional , Y , H , R,I );
%          end
%          S = R + H*P_minus*H';
%          K = P_minus*H'/(diag(diag(S),0));                % %Computed Kalman gain
%          X = X_hat_minus + (K * (Y - H*X_hat_minus'))';
% % %          P = P_minus - K * S * K';
%          P = P_minus - K * H * P_minus;
%          P = (P+P')/2;

%         Simulation.Output.Kalman_mtx.update_counter = Simulation.Output.Kalman_mtx.update_counter +1;
%%  Q = K*H*C/N*H'*K';
% Simulation.Output.Kalman_mtx.gain(Simulation.Output.Kalman_mtx.update_counter,:)=[K(1,1) K(1,2) K(2,1) K(2,2) K(3,1) K(3,2) K(4,1) K(4,2) K(5,1) K(5,2) K(6,1) K(6,2)];
end