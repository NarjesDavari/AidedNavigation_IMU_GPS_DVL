function [Simulation,C]= Covariance_innovation(Simulation,N)

  update_counter=Simulation.Output.Kalman_mtx.update_counter;
  meanV = mean(Simulation.Output.Kalman_mtx.V(update_counter-N+1:update_counter,:));
  H = Simulation.Output.Kalman_mtx.H;
  C=0;
  P_=0;
  if update_counter<=N
     for i=1:N
         C=C+ ((Simulation.Output.Kalman_mtx.V(i,:)-meanV)'*(Simulation.Output.Kalman_mtx.V(i,:)-meanV));
     end
  else
     for j=update_counter-N:1:update_counter
      C = C+((Simulation.Output.Kalman_mtx.V(j,:))-meanV)'*((Simulation.Output.Kalman_mtx.V(j,:)-meanV));
     end
  end