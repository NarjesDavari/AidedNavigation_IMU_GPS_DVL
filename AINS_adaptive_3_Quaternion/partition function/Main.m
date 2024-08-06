disp(' ')
clear all
close all
clc
 



disp(['--------------------------------------------------------------------------------------------------------------- '])
disp('Code related to the paper')
disp('L. Martino, V. Elvira, D. Luengo, J. Corander, "Layered Adaptive Importance Sampling", Statistics and Computing (accepted; to appear), 2016')
disp(' ')
disp(' More specifically, this is a preliminary code of')
disp('the Parallel Interacting Markov Adaptive Importance Sampling (PI-MAIS) Algorithm ')
disp(' ')




DIM=2; %%%% DIMENSION OF THE STATE SPACE
typeTarget=1;%%%% 1-2-3 %%% aux variable for selecting the posterior target pdf 

if typeTarget==1
   DIM=2; 
end


load('Real_Measurement_test1_22th_2','Real_Measurement')
x=Real_Measurement.DVL(3570:3615,2);
s1=median(x)+randn(1,1)*std(x);
s2=median(x)-randn(1,1)*std(x);
j=1;
% for i=1:length(x)
%     if x(i)<s1 || x(i)>s2
%     else
%         y(j)=x(i);
%         j=j+1;
%     end
% end
 y=x;       

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Parameters of the algorithm %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N=200; %%% Number of proposal pdfs; Number of chains in the upper layer
T =500;%%% total number of iterations
M=2; %%% samples from each proposal at each iteration
EvTot =N*((M+1)*T); %%% total number of evaluations of the target pdf
disp(['--------------------------------------------------------------------------------------------------------------- '])
disp(['Dimension of the space =', num2str(DIM)])
disp(['Number of parallel chains, N=', num2str(N)])
disp(['Number of iterations, T=', num2str(T)])
disp(['Number of samples per proposals and per iterations, M=',num2str(M)])
disp(['Total number of target evaluations E= ',num2str(EvTot)])  
  
%%%%%% scale parameters %%%%
SIG_IS_LowerLayer = std(y); %%% std LL
SIG_MCMC_UpperLayer = std(y); %%$ std UL
SIG_MCMC_UpperLayer_vec = ones(DIM,1)*SIG_MCMC_UpperLayer; 

%%%%%% outputs %%%%
TypeOutput=1; %%% 0=only estimations expected value; 1= also all the pair {x,w}'s


%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% main file %%%%%%%%%
[Est_tot,NormEst,mu,muint,x_tot,W_tot]=PIMAIS_v03(N,T,M,SIG_MCMC_UpperLayer_vec,SIG_IS_LowerLayer,DIM,typeTarget,TypeOutput,y);
%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%

  disp(['--------------------------------------------------------------------------------------------------------------- '])
  disp('True expected value of the target')
  if typeTarget==1
      disp([1.6 1.4])
  elseif typeTarget==2
     disp(-6*ones(1,DIM))
   elseif typeTarget==3
        disp(-5*ones(1,DIM))
  end

  disp('Estimation of the expected value of the target')
 disp(Est_tot')
 disp(['Estimation of the normalizing constant of the target (true value=1) = ', num2str(NormEst)])
  if TypeOutput==1
     disp(['(only one check- see below - another computation way) '])
     disp(['Estimation of the normalizing constant of the target (true value=1) = ', num2str((M*N*T)./sum(sum(W_tot)))])
     disp(['---------------------------------------------------------------------------------------------------------------- '])
     disp(['the size of the vector W_tot is M*T \times N = ', num2str(M*T), 'x' ,num2str(N)])
  end
  
     plot(muint(1,:),muint(2,:),'gs') 
     hold on
     plot(mu(1,:),mu(2,:),'ro')   
     set(gca,'Fontsize',20)
     title('Marginal distributions of the means (first two components)')
     
     legend('Initial distr.','Final  distr.','Location','NorthWest')
       if typeTarget==1
      axis([-20 20 -20 20])
  elseif typeTarget==2
     axis([-30 20 -30 20])
   elseif typeTarget==3
     axis([-30 20 -30 20])
  end