

clear; clc;
tic
% load('Real_Measuremet_test1_22th_2','Real_Measurement')
% x=Real_Measurement.DVL(3570:3615,2); %% 
load('Velocity','v')
x=v(3490:3580,2); %% 

l=4; %%number of units in input layer
k=1; %%number of units in output layer
m=2; %% number of hidden layer
n=l;%% number of units in hidden layer = number of units in input layer  ???
w=l;

%%% criterial stopping
epi_th = 10^-3;
e_th = 10^-3;
%% 

%%%%%%%%%%%%
Lr=.01;

 counter (1:length(x),1)=0; 
count_outlier=0;
norm_DE2=[];
for i=1:length(x)-w
   
     u = 0.001*randn(1,1); %%% value for u0
     h = 0.001*randn(1,1); %%% value for h0
    
    b_uu = 0.001*randn(1,1);
    W_uu = 0.001*randn(n,1); %% weigth between hidden layer u_t-1 and u_t
%     DW_uu = zeros(n,1);
    
    b_hu = 0.001*randn(1,1);
    W_hu = 0.001*randn(n,n); %% weigth between hidden layer h_t and uhat_t
%     DW_hu = zeros(n,1);
   
   
    b_ih = 0.001*randn(1,1);
    W_ih = 0.001*randn(n,n); %% weigth between input layer i_t and h_t
    W_hh = 0.001*randn(n,1); %% weigth between input layer h_t-1 and h_t
%     DW_hh = zeros(n,1);
%%%% initila value
epi = 1; e = 1;
m_hu = 0; m_bhu=0; m_ih=zeros(n,n);m_bih=0;m_hh=zeros(n,n);m_uu=0;m_buu=0;
v_hu = 0; v_bhu=0; v_ih =zeros(n,n); v_bih=0;v_hh=zeros(n,n);v_uu=0;v_buu=0;
alpha_hu = 0.002; alpha_bhu=0.002; alpha_ih=0.002;alpha_bih=0.002; alpha_hh=0.002;alpha_uu=0.002; alpha_buu=0.002;
beta_coeff1 = 0.9; beta_coeff2=0.999;
beta1_hu = beta_coeff1; beta1_bhu=beta_coeff1; beta1_ih=beta_coeff1;  beta1_bih=beta_coeff1; beta1_hh=beta_coeff1; beta1_uu=beta_coeff1; beta1_buu=beta_coeff1; 
beta2_hu = beta_coeff2; beta2_bhu=beta_coeff2; beta2_ih=beta_coeff2;  beta2_bih=beta_coeff2; beta2_hh=beta_coeff2; beta2_uu=beta_coeff2; beta2_buu=beta_coeff2; 
% Lr_hu = 0.001;
regular_coeff = 10^(-8);

W_hu_new_bar=0;
u_hu=0;
%%%%%%%%%
kk=2;
count_dec=0;
flag_Learning=1;
energ3 (i,kk-1)=100;
    I = x(i:w+i-1);
%     while flag_Learning==1 
        
      for kk=2:1000
        %% Forward Propagation
        for k=2:w+1
            u(k,1) = ActiveF_diff((W_uu(k-1,1)* u(k-1,1) + b_uu),1);
            h(k,1) = ActiveF_diff((W_ih(k-1,:)* I + W_hh(k-1,1)* h(k-1,1) + b_ih),1);
        end
        u = u(2:end,1);
        h = h(2:end,1);
        u_hat = ActiveF_diff ((W_hu * h + b_hu *ones(n,1)),1); %%%logsig(h_i); %% output from neuron of hidden layer
        E_mi = u-u_hat;
        E_m = exp(0.5* sum(E_mi.^2));
        %% Backward Propagation
        DE_Whu = E_m * sum((E_mi).* ActiveF_diff (u_hat,2).* h);
        m_hu = beta1_hu * m_hu + (1-beta1_hu)*DE_Whu;
        m_hu_hat = m_hu/(1-beta1_hu^(kk-1));
        v_hu = beta2_hu * v_hu + (1-beta2_hu)*DE_Whu.^2;
        v_hu_hat = v_hu/(1-beta2_hu^(kk-1));
        alpha_t_hu = alpha_hu* (sqrt(1-beta2_hu^(kk-1)))/(1-beta1_hu^(kk-1));%% learning rate
        %         u_hu = max(beta2_hu*u_hu , abs(DE_Whu));
        %         W_hu_new =  W_hu - (alpha_hu/(1-beta1_hu^(kk-1))).*m_hu./u_hu;
        %         W_hu_new =  W_hu- alpha_t_hu* m_hu/(sqrt(v_hu)+regular_coeff);
        W_hu_new =  W_hu - alpha_hu* m_hu_hat/(sqrt(v_hu_hat+regular_coeff));
        %         W_hu_new_bar = beta2_hu * W_hu_new_bar + (1-beta2_hu)*W_hu_new;
        %         W_hu_new = W_hu_new_bar /(1-beta2_hu^(kk-1));
        %         W_hu_new = W_hu - Lr*abs(DE_Whu); %%% new weight (updated value) %% Stochastic gradeint descent
        Lr_hu (i,kk-1)= alpha_t_hu;
        
        DE_bhu = E_m * sum((E_mi).* ActiveF_diff (u_hat,2));
        m_bhu = beta1_bhu * m_bhu + (1-beta1_bhu)*DE_bhu;
        m_bhu_hat = m_bhu/(1-beta1_bhu^(kk-1));
        v_bhu = beta2_bhu * v_bhu + (1-beta2_bhu)*DE_bhu.^2;
        v_bhu_hat = v_bhu/(1-beta2_bhu^(kk-1));
        alpha_t_bhu = alpha_bhu* (sqrt(1-beta2_bhu^(kk-1)))/(1-beta1_bhu^(kk-1));%% learning rate
        %         b_hu_new = b_hu - alpha_t_bhu* m_bhu/(sqrt(v_bhu)+regular_coeff);
        b_hu_new = b_hu - alpha_bhu* m_bhu_hat/(sqrt(v_bhu_hat)+regular_coeff);
        %         b_hu_new = b_hu - Lr*abs(DE_bhu); %%% new bias (updated value) %% Stochastic gradeint descent
        
        DE_Wih = E_m *W_ih .* sum((E_mi).* ActiveF_diff (u_hat,2).* ActiveF_diff (h,2) .* I);
        m_ih = beta1_ih * m_ih + (1-beta1_ih)*DE_Wih;
        m_ih_hat = m_ih/(1-beta1_ih^(kk-1));
        v_ih = beta2_ih * v_ih + (1-beta2_ih)*DE_Wih.^2;
        v_ih_hat = v_ih/(1-beta2_ih^(kk-1));
        alpha_t_ih = alpha_ih* (sqrt(1-beta2_ih^(kk-1)))/(1-beta1_ih^(kk-1)); %% learning rate
        %         W_ih_new =  W_ih - alpha_t_ih* m_ih ./(sqrt(v_ih)+regular_coeff);
        W_ih_new =  W_ih - alpha_ih* m_ih_hat ./(sqrt(v_ih_hat)+regular_coeff);
        %         W_ih_new = W_ih - Lr*abs(DE_Wih); %%% new weight (updated value) %% Stochastic gradeint descent
        
        DE_bih = E_m *W_ih .* sum((E_mi).* ActiveF_diff (u_hat,2).* ActiveF_diff(h,2));
        m_bih = beta1_bih * m_bih + (1-beta1_bih)*norm(DE_bih);
        m_bih_hat = m_bih/(1-beta1_bih^(kk-1));
        v_bih = beta2_bih * v_bih + (1-beta2_bih)*norm(DE_bih)^2;
        v_bih_hat = v_bih/(1-beta2_bih^(kk-1));
        alpha_t_bih = alpha_bih* (sqrt(1-beta2_bih^(kk-1)))/(1-beta1_bih^(kk-1));%% learning rate
        %         b_ih_new = b_ih - alpha_t_bih* m_bih./(sqrt(v_bih)+regular_coeff);
        b_ih_new = b_ih - alpha_bih* m_bih_hat./(sqrt(v_bih_hat)+regular_coeff);
        %         b_ih_new = b_ih - Lr*abs(norm(DE_bih)); %%% new bias (updated value) %% Stochastic gradeint descent
        
        DE_Whh = E_m *W_ih .* sum((E_mi).* ActiveF_diff (u_hat,2).* ActiveF_diff (h,2) .* h);
        m_hh = beta1_hh * m_hh + (1-beta1_hh)*DE_Whh;
        m_hh_hat = m_hh/(1-beta1_hh^(kk-1));
        v_hh = beta2_hh * v_hh + (1-beta2_hh)*DE_Whh.^2;
        v_hh_hat = v_hh/(1-beta2_hh^(kk-1));
        alpha_t_hh = alpha_hh* (sqrt(1-beta2_hh^(kk-1)))/(1-beta1_hh^(kk-1));%% learning rate
        %         W_hh_new =  W_hh- sum(alpha_t_hh* m_hh ./(sqrt(v_hh)+regular_coeff),2); %%% or we can use norm instead of sum
        W_hh_new =  W_hh- sum(alpha_hh* m_hh_hat ./(sqrt(v_hh_hat)+regular_coeff),2);
        %         W_hh_new = W_hh - Lr*abs(sum(DE_Whh,2)); %%% new weight (updated value) %% Stochastic gradeint descent
        
        for k=2:w+1
            h(k,1) = ActiveF_diff((W_ih_new(k-1,:) * I + W_hh_new(k-1,1)* h(k-1,1) + b_ih_new),1);
        end
        h=h(2:end,1);
        u_hat= ActiveF_diff((W_hu_new * h + b_hu_new*ones(n,1)),1);
        C_mi  = u-u_hat;
        DC_Wuu = sum(C_mi.* ActiveF_diff(u,2) .* u);
        
        m_uu = beta1_uu * m_uu + (1-beta1_uu)*DC_Wuu;
        m_uu_hat = m_uu/(1-beta1_uu^(kk-1));
        v_uu = beta2_uu * v_uu + (1-beta2_uu)*DC_Wuu.^2;
        v_uu_hat = v_uu/(1-beta2_uu^(kk-1));
        alpha_t_uu = alpha_uu* (sqrt(1-beta2_uu^(kk-1)))/(1-beta1_uu^(kk-1));%% learning rate
        %         W_uu_new =  W_uu - alpha_t_uu* m_uu /(sqrt(v_uu)+regular_coeff);
        W_uu_new =  W_uu - alpha_uu* m_uu_hat /(sqrt(v_uu_hat)+regular_coeff);
        %        W_uu_new = W_uu - Lr* abs (DC_Wuu); %%% new weight (updated value) %% Stochastic gradeint descent
        
        DC_buu = sum(C_mi .* ActiveF_diff(u,2));
        m_buu = beta1_buu * m_buu + (1-beta1_buu)*DC_buu;
        m_buu_hat = m_buu/(1-beta1_buu^(kk-1));
        v_buu = beta2_buu * v_buu + (1-beta2_buu)*DC_buu.^2;
        v_buu_hat = v_buu/(1-beta2_buu^(kk-1));
        alpha_t_buu = alpha_buu* (sqrt(1-beta2_buu^(kk-1)))/(1-beta1_buu^(kk-1));%% learning rate
        %         b_uu_new = b_uu - alpha_t_buu* m_buu/(sqrt(v_buu)+regular_coeff);
        b_uu_new = b_uu - alpha_buu* m_buu_hat/(sqrt(v_buu_hat)+regular_coeff);
        %         b_uu_new = b_uu - Lr* abs(DC_buu); %%% new bias (updated value) %% Stochastic gradeint descent
        %%
        epi(i,kk) = norm(W_uu_new - W_uu) + norm(b_hu_new - b_hu) + norm(W_hu_new - W_hu) + (b_hu_new - b_hu);
        
        y1 = h; 
         for k =2:w
        y1 (k,1) = log(1+ exp( W_hh_new(k-1,1) * y1(k-1,1) + W_ih_new(k,1) * I(k,1)+ b_ih_new ));
         end
%         y1 = log(1+exp(W_ih_new *I +b_ih_new +W_hh_new .* h));
%         y2 = log(1+exp(W_uu_new .* u + b_uu_new));
%         y3 = log(1+exp(W_hu_new * h + b_hu_new));
        
        energ(i,kk) = (0.5* norm((I-u_hat).^2) - sum(y1)); %%%
        %         partition_f = 1;
        %         e = exp(-(energ))/partition_f;
        grad_DE(i,kk-1) =norm(( sum(u-u_hat)/norm(u-u_hat)/2-(W_uu_new.*exp(W_uu_new.*u + b_uu_new))/(1+exp(W_uu_new.*u + b_uu_new))/log(10)).^2);
        energ2(i,:)= exp(-1* (b_hu_new *u_hat + W_hu_new *u_hat.*h + b_uu_new.*u + u.*W_uu_new .*u));
        
        energ3(i,kk) = mean((u-u_hat).^2);%%% reconstruction error ???
        if kk>100
            if (energ3(i,kk-1)-energ3(i,kk-2)) < 0 
                if (energ3(i,kk)-energ3(i,kk-1)) > 0 
                    if energ3(i,kk-1)< min(energ3(i,2:kk)) 
                        flag_Learning=0;
                    end
                elseif abs(energ3(i,kk)-energ3(i,kk-1)) < 10^(-7)
                    count_dec = count_dec+1;
                end
            else
                flag_Learning=1;
            end
        end
    if kk>10000 || count_dec==10
        flag_Learning=0;
    end
     kk= kk+1;       
        W_hu = W_hu_new;
        b_hu = b_hu_new;
        W_ih = W_ih_new;
        b_ih = b_ih_new;
        W_hh = W_hh_new;
        W_uu = W_uu_new;
        b_uu = b_uu_new;
        h(1,1)= h(end,1);
        u(1,1)= u(end,1);
                counter(i)=counter(i)+1;
                count_decF(i)=count_dec;
      end
     x_est(i,1) = u(end);
 energ_timeW (i,1) = mean (energ(i,2:end));  
  
c_max =2; c_min=1;
coef_out =1; %%c_max-rand(1)*c_min;
score(i,1)=mean(energ3(i,2:end-1));
if i>1 
    if score(i)< coef_out*mean(score(1:i-1,1))
    x_out(i,1)=x(i+w-1);
    else
    x_out(i,1)=x(i+w-2);
    end
else
    x_out(i,1)=x(i+w-1);
end
%       energy(i,1)=energ;
%       energy2(i,1)=norm(energ2);
      coeff_DE = 50;
      norm_DE(:,i)=[(norm(DE_Whu))^2; (norm(DE_bhu))^2;(norm(DE_Wih))^2; (norm(DE_bih))^2; (norm(DE_Whh))^2; (norm(DC_Wuu))^2;(norm(DC_buu))^2];
    s_de(i,1)=sum(norm_DE(:,i));
%     if i==1
%         norm_DE2= [norm_DE2 norm_DE(:,i)];
%     end
%     if i>1 && norm_DE(1,i)> coeff_DE*median(norm_DE2(1,:))
%         count_outlier =count_outlier+1;
%         fprintf ('there is a outlier %f\n ',count_outlier)
%        norm_DE2= [norm_DE2 norm_DE(:,i-1)];
%        x_out(i)= x_out(i-1);
%     elseif i>1
%         norm_DE2= [norm_DE2 norm_DE(:,i)];
%         x_out(i)= u_hat(end);

%     end
    
%% 
%% 
st(:,i)=1-(u_hat.*u)./abs(u);
L(i,1)=(1/2/pi/var(st(:,i)))*exp(-((st(end,i))-mean(st(:,i)))^2/2/var(st(:,i)));
%% 

end
toc
% figure;plot(score)
% figure;plot(x(w+1:end));hold on;plot(x_out,'r')
%%%%%%%%% Gradient cheking
% W=[W_ih_new;W_ho_new];
% Wpluse=W;
% Wminus=W;
% Epsilon=1e-4;
% for i=1:length(W)
% Wpluse (i)= Wpluse (i)+ Epsilon;
% Wminus (i)= Wminus (i) - Epsilon;
% gradApprox (i) = (costFun (Wpluse) - costFun (Wminus))/(2*Epsilon); %%%% numerical estimation gradient descent
% end
% if gradApprox ==Dvec
%     
% end



