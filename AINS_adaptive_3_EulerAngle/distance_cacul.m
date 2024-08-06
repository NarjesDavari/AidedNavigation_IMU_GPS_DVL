%Computation of travelled distance at timestep i with respect to initial
%point
function [ Simulation ] = distance_cacul( Simulation , ave_sample )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %P(:,1):position along north(Xn)
    %P(:,2):position along east(Xe)
    %P(:,3):position along up(h)
    P(:,1)=Simulation.Input.Measurements.Ref_Pos(ave_sample:end,2);
    P(:,2)=Simulation.Input.Measurements.Ref_Pos(ave_sample:end,3);
    P(:,3)=Simulation.Input.Measurements.Ref_Pos(ave_sample:end,4);
    
    %Creation of space in memory
    Simulation.Input.Path.ds_i=zeros(size(P,1),1);
    Simulation.Input.Path.s_i=zeros(size(P,1),1);    
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xx=P(:,1);
    yy=P(:,2);
    zz=P(:,3);
    %
    dx=diff(xx);
    dy=diff(yy);
    dz=diff(zz);
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sx_i=cumsum(abs(dx));
    sy_i=cumsum(abs(dy));
    if dz==0
        sz_i=mean(zz);
    else
    sz_i=cumsum(abs(dz));
    end
    Simulation.Input.Path.sx_i=zeros(size(P,1),1);
    Simulation.Input.Path.sy_i=zeros(size(P,1),1);
    Simulation.Input.Path.sz_i=zeros(size(P,1),1);
    
    Simulation.Input.Path.sx_i=sx_i; 
    Simulation.Input.Path.sy_i=sy_i; 
    Simulation.Input.Path.sz_i=sz_i; 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    dP=[dx dy dz];
    sqr_dP=dP.^2;
    ds_i=sqrt(sum(sqr_dP,2));%distance of P(i+1) from P(i) in main(designed) Path
    s_i=cumsum(ds_i);%Travelled distance at timestep i with respect to initial point
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Simulation.Input.Path.ds_i=ds_i;
    Simulation.Input.Path.s_i=s_i;      
    
end

