function [f,logf]=evaluate_target3(x,type,dim)


if type==1
    
    
         mu1=[-10 -10];
      SIGMA1 = [2 0.6; 0.6 1];
      mu2=[0 16];
      SIGMA2 = [2 -0.4;-0.4 2];
      mu3=[13 8];
      SIGMA3 = [2 0.8;0.8 2];
      mu4=[-9 7];
      SIGMA4 = [3 0; 0 0.5];
      mu5=[14 -14];
      SIGMA5 = [2 -0.1;-0.1 2];
      
    f1=1/5*mvnpdf(x',mu1,SIGMA1);
    f2=1/5*mvnpdf(x',mu2,SIGMA2);
    f3=1/5*mvnpdf(x',mu3,SIGMA3);
    f4=1/5*mvnpdf(x',mu4,SIGMA4);
     f5=1/5*mvnpdf(x',mu5,SIGMA5);
    
    f=f1+f2+f3+f4+f5;
    
    logf=log(f');
    
    
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
elseif type==2 
    
  sigma_target=2;  
  
    
     mu(1,:)=-10*ones(1,dim);
    Sigma(:,:,1) =sigma_target^2*eye(dim);
    mu(2,:)=10*ones(1,dim);
    Sigma(:,:,2) =sigma_target^2*eye(dim);
    mu(3,:)=-20*ones(1,dim);
    %  mu(3,:)=[1 2 3 4 5 5 4 3 2 1];
    Sigma(:,:,3) =sigma_target^2*eye(dim);
    for k = 1:3
        f(k,:)=1/3*mvnpdf(x',mu(k,:),Sigma(:,:,k)); % N last samples (new samples for all proposals)
    end
    f=sum(f);
    logf=log(f);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
elseif type==3
   
      
    Sigma(:,:,1) =1*eye(dim);
    mu(1,:)=10*ones(1,dim);
    Sigma(:,:,2) =5*eye(dim);
    mu(2,:)=-20*ones(1,dim);
    for k = 1:2
        f(k,:)=1/2*mvnpdf(x',mu(k,:),Sigma(:,:,k)); % N last samples (new samples for all proposals)
    end
    f=sum(f);
    logf=log(f);
    
end