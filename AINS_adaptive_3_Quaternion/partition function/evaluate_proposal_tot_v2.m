function [fp2,fp3]=evaluate_proposal_tot_v2(x,mu,sig,N,M)
%%%% fp2= provide denominator for spatial deterministic mixture = sum_{n=1}^N q_n(x_n)
%%%% fp3= provide stardard denominator =q_n(x_n)





v_aux = repmat(1:N,[M 1]);
v_aux = v_aux(:)';

for i=1:N*M %%%% for each samples in this case
    z=repmat(x(:,i),1,N);
    
    This_x_in_All_Proposal_Pdfs =exp(-0.5 * ((z - mu)./sig).^2) ./ (sqrt(2*pi) .* sig);
 
    
    fp1=prod(This_x_in_All_Proposal_Pdfs);

   
    fp2(i)=1/N*sum(fp1); %%%% evaluating spatial mixture
    
    fp3(i)=fp1(v_aux(i));
   
    

end


