
%%%%%% time update in information filter
function [Simulation,dX,P,alpha,beta] = predict_IF_VB (Simulation,I,landa_P)

    Y=Simulation.Output.Kalman_mtx.Y; %%% information matrix
    y=Simulation.Output.Kalman_mtx.y; %%% information state
    A=Simulation.Output.Kalman_mtx.A;
    Qc=Simulation.Output.Kalman_mtx.QQc; 
    G=Simulation.Output.Kalman_mtx.G;
    
    alpha=Simulation.Output.Kalman_mtx.alpha_VB;
    beta=Simulation.Output.Kalman_mtx.beta_VB;
    rho=1;%%%%????(1-exp(-4));
    
    
    Y = (landa_P*(A* Y^(-1) * A' + G* Qc* G'))^(-1);
    L = Y * A* Y^(-1); %% information propagation coefficent
    y = L* y;

    alpha = rho*alpha;
    beta = rho*beta;
    P=Y^(-1);
    dX=zeros(15,1); %%% 
   Simulation.Output.Kalman_mtx.Y = Y; %%% information matrix
   Simulation.Output.Kalman_mtx.y = y; 

end