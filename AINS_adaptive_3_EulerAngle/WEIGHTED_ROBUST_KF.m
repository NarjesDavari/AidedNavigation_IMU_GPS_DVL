
function [dX,P]=WEIGHTED_ROBUST_KF(V,H,R,P,dX)
Q=Simulation.Output.Kalman_mtx.Q;
alpha=1e-6;
beta=1e-6;
small=1e-6;
oldP = P;
omega= V'*V+trace(H'*H+oldP);
weight=(alpha+0.5)/(beta+ (0.5/(R+small))*omega);
S=H*Q*H' +R/(weight+small);

K = P*H'/S;
P = P - K*H*P; 
dX = dX + K * ( dz-H*dX);
end
