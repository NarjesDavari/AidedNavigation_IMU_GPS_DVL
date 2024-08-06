

function [Simulation,X,P] = EKF_update_2(Simulation, X_hat_minus , Y , H , R,I)
 
  %
  % update step
  P_plus_k = Simulation.Output.Kalman_mtx.P_plus;
  X_pluse_k = Simulation.Output.Kalman_mtx.X_plus;
  A = Simulation.Output.Kalman_mtx.A;
  Q = Simulation.Output.Kalman_mtx.Q;
  
  S = (R + H*P_plus_k*H');
  K = P_plus_k*H'/S;
  X = (X_hat_minus' + (K * (Y - H*X_pluse_k')))';
  P = A*(eye(15) - K * H) * P_plus_k * A' +Q;

