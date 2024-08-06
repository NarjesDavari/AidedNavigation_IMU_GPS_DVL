function [Est_tot,NormEst,mu,muint,x_tot,W_tot]=PIMAIS_v03(N,T,M,sig_UL,SIG_LL,DIM,typeTarget,TypeOutput,y)
%%% Preliminary code of the Parallel Interacting Markov Adaptive Importance Sampling (PI-MAIS) Algorithm
%%% see L. Martino, V. Elvira, D. Luengo, J. Corander, "Layered Adaptive Importance Sampling", Statistics and Computing (accepted; to appear), 2016


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% INITIALIZATION of different variables%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    mu = mean(y)+std(y)*rand(DIM,N); %%%% random initialization of the means   
  %%%%   sig_LL=1+9*rand(DIM,N); %%% RAMDOM CHOIC
  %%%%   sig_LL=SIG_LL*abs(randn(DIM,N)); %%% RAMDOM CHOICE
        sig_LL=SIG_LL*ones(DIM,N); %%% std LOWER LEVEL
 
muint=mu;

%%%% INITIALIZATION %%%%
S1=0;
Stot=0;
Est_part=zeros(DIM,1);
Est_tot=zeros(DIM,1);

countAllSample=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% MAIN LOOP              %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t=1:T
    
    %%% just for displaying something during the wait  
    if mod(t,300)==0
      disp([num2str(min([fix(t*100/T) 100])) '%'])        
    end
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% ADAPTING THE MEANS VIA MCMC - UPPER LAYER %%%%
    mu=MCMC_parallel_HD(mu,sig_UL,N,typeTarget,DIM);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%  LOWER LAYER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%% Drawing M samples for each proposal%%%%%%
 
        mu_all = repmat(mu,1,M);
        sig_all = repmat(sig_LL,1,M); 
        
        x=mu_all+sig_all.*randn(DIM,N*M); %%%%% Samples in the Lower Layer (Gaussian proposal pdfs)
        
        
        [f,logf]= evaluate_target3(x,typeTarget,DIM); %%%% evaluation target pdf
  
        [fp2,fp]=evaluate_proposal_tot_v2(x,mu,sig_LL,N,M); %%%% evaluation proposal pdfs
        logfp2=log(fp2+10^(-320)); %%% "+10^(-320)" to avoid numerical problems
     
        
     
        logw=logf-logfp2;
        w=exp(logw)+10^(-320); %%% "+10^(-320)" to avoid numerical problems
     
        S1=sum(w);
        wn=w/S1; %%% normalized weights
            
        WNrep=repmat(wn,DIM,1);
        Est_part=sum(WNrep.*x,2); %%% Estimation at this iteration
        Est_tot=(Stot*Est_tot+S1*Est_part)/(Stot+S1); %%% Global Estimation 
        Stot=Stot+S1;  
        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %%%% ALL the samples ALL weights %%%%%%
    if TypeOutput==1   %%%%% output also all the pair "samples and weights" 
        countAllSample=countAllSample+1;
        x_tot{countAllSample}=x; %%%% samples - lower layers at t-th iteration
        W_tot(countAllSample,:)=w; %%% unnormalized weights at t-th iteration
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        


    
    
    
    

end %%%% end--- for t=1:T

%%%%  ESTIMATION OF THE NORMALIZING CONSTANT 
NormEst=1/(1/(N*M*T)*Stot); %%%%  MARGINAL LIKELIHOOD ESTIMATION

 if TypeOutput==0   %%%%% outputonly expected value      
        x_tot='You set TypeOutput=0; change to TypeOutput=1 ';
        W_tot='You set TypeOutput=0; change to TypeOutput=1 '; 
    end
   
