function xend=MCMC_parallel_HD(xpop,sigprop,N,typeTarget,M,y)

% M=length(sigprop);
length(sigprop);
% M
% keyboard
x{1}=xpop';

if sigprop == 0 % Random sigmas
    sigma_matrix = mean(y)+std(y)*rand(N,M);
else % Same sigmas
    sigma_matrix = repmat(sigprop',N,1);
end

Tchain=1;
for i=2:Tchain+1
    
    x{i}=x{i-1}+sigma_matrix.*randn(N,M);
    f = evaluate_target3(x{i-1}',typeTarget,M);

    Vbef = log(f);   
    f = evaluate_target3(x{i}',typeTarget,M);
    Vnow = log(f);
    
    [d1,d2]=size(Vbef);
    if d1>d2
        Vbef=Vbef';
        Vnow=Vnow';
    end
    rho=exp(Vnow-Vbef);
    
    alpha=min([ones(1,N);rho]);
    u=rand(1,N);
    test=(u<=alpha);
    vec_aux=(x{i}-x{i-1}).*repmat(test',1,M);
    x{i}=x{i-1}+vec_aux;
    
end

xend=x{end}';