

clear; clc;
tic
load('Velocity','Velocity')
x_traning_set=Velocity.traning_set; %%(3400:3800,2); %% 
x_testing_set = Velocity.testing_set(3400:3800,2);


l=4; %%number of units in input layer
k=1; %%number of units in output layer
m=2; %% number of hidden layer
n=l;%% number of units in hidden layer = number of units in input layer  ???
w=l;

%%% criterial stopping
epi_th = 10^-3;
e_th = 10^-3;
%% 
    u = 0.001*randn(1,1); %%% value for u0
    b_uu = 0.001*randn(1,1);
    W_uu = 0.001*randn(n,1); %% weigth between hidden layer u_t-1 and u_t
%   DW_uu = zeros(n,1);
    
    b_hu = 0.001*randn(1,1);
    W_hu = 0.001*randn(n,1); %% weigth between hidden layer h_t and uhat_t
%     DW_hu = zeros(n,1);
   
    h = 0.001*randn(1,1); %%% value for h0
    b_ih = 0.001*randn(1,1);
    W_ih = 0.001*randn(n,1); %% weigth between input layer i_t and h_t
    W_hh = 0.001*randn(n,1); %% weigth between input layer h_t-1 and h_t
%   DW_hh = zeros(n,1);
counter=0;
%%%% initila value
epi = 1; e = 1;
m_hu = 0; m_bhu=0; m_ih=zeros(w,1);m_bih=0;m_hh=zeros(w,1);m_uu=0;m_buu=0;
v_hu = 0; v_bhu=0; v_ih =zeros(w,1); v_bih=0;v_hh=zeros(w,1);v_uu=0;v_buu=0;
alpha_hu = 0.01; alpha_bhu=0.01; alpha_ih=0.01;alpha_bih=0.01; alpha_hh=0.01;alpha_uu=0.01; alpha_buu=0.01;
beta_coeff1 = 0.9; beta_coeff2=0.999;
beta1_hu = beta_coeff1; beta1_bhu=beta_coeff1; beta1_ih=beta_coeff1;  beta1_bih=beta_coeff1; beta1_hh=beta_coeff1; beta1_uu=beta_coeff1; beta1_buu=beta_coeff1; 
beta2_hu = beta_coeff2; beta2_bhu=beta_coeff2; beta2_ih=beta_coeff2;  beta2_bih=beta_coeff2; beta2_hh=beta_coeff2; beta2_uu=beta_coeff2; beta2_buu=beta_coeff2; 

%%%%%%%%%%%%
% Lr=.1;
count_outlier=0;
count (1:length(x_traning_set),1)=0; 
norm_DE2=[];
%% 
for i=1:length(x_traning_set)-w

% Lr_hu = 0.001;
regular_coeff = 10^(-8);
%%%%%%%%%
kk=2;
count_dec=0;
flag_Learning=1;
energ3 (i,kk-1)=1;
const_lr =.1;
%% 
    k1=1; E_testing(1,1)=10;
    I_testing_set = x_testing_set(i:w+i-1);
    I_traning_set = x_traning_set(i:w+i-1);
%     if i>2
%         I_traning_set(end)= u(end);
%     end
%     while  flag_Learning==1
%         for t=10000:-.5:.5 %% anealing schadule
%             Lr=const_lr/(t);
     for kk=2:500
        %% Forward Propagation
        for k=2:w+1
            u(k,1) = ActiveF_diff(W_uu(k-1,:)* u(k-1,1) + b_uu,1);
            h(k,1) = ActiveF_diff(W_ih(k-1,:)* I_traning_set(k-1,1) + W_hh(k-1,:)* h(k-1,1) + b_ih,1);
        end
        u = u(2:end,1);
        h = h(2:end,1);
        u_hat = ActiveF_diff(W_hu .* h + b_hu,1); %%%logsig(h_i); %% output from neuron of hidden layer
        
        E_mi = u-u_hat;  
        E_m = exp(0.5* sum(E_mi.^2));
        %% Backward Propagation
        DE_Whu = E_m * sum((E_mi).* ActiveF_diff (u_hat,2).* h);
        m_hu = beta1_hu * m_hu + (1-beta1_hu)*DE_Whu;
        v_hu = beta2_hu * v_hu + (1-beta2_hu)*DE_Whu.^2;
        alpha_t_hu = alpha_hu* (sqrt(1-beta2_hu^(kk)))/(1-beta1_hu^(kk));
        W_hu_new =  W_hu - alpha_t_hu* m_hu/(sqrt(v_hu)+regular_coeff);
%         W_hu_new = W_hu - Lr*abs(DE_Whu);
%         W_hu_new = W_hu - Lr*abs(DE_Whu); %%% new weight (updated value) %% Stochastic gradeint descent
        
        DE_bhu = E_m * sum((E_mi).* ActiveF_diff (u_hat,2));
        m_bhu = beta1_bhu * m_bhu + (1-beta1_bhu)*DE_bhu;
        v_bhu = beta2_bhu * v_bhu + (1-beta2_bhu)*DE_bhu.^2;
        alpha_t_bhu = alpha_bhu* (sqrt(1-beta2_bhu^(kk)))/(1-beta1_bhu^(kk));
        b_hu_new = b_hu - alpha_t_bhu* m_bhu/(sqrt(v_bhu)+regular_coeff);
%         ss1(kk)=alpha_t_bhu* m_bhu(:,kk)/(sqrt(v_bhu(:,kk))+regular_coeff);
%         b_hu_new = b_hu - Lr*abs(DE_bhu); %%% new bias (updated value) %% Stochastic gradeint descent
        
        DE_Wih = E_m *W_ih .* sum((E_mi).* ActiveF_diff (u_hat,2).* ActiveF_diff (h,2) .* I_traning_set);
        m_ih = beta1_ih * m_ih + (1-beta1_ih)*DE_Wih;
        v_ih = beta2_ih * v_ih + (1-beta2_ih)*DE_Wih.^2;
        alpha_t_ih = alpha_ih* (sqrt(1-beta2_ih^(kk)))/(1-beta1_ih^(kk));
        W_ih_new =  W_ih - alpha_t_ih* m_ih./(sqrt(v_ih)+regular_coeff);
%         W_ih_new = W_ih - Lr*abs(DE_Wih); %%% new weight (updated value) %% Stochastic gradeint descent
        
        DE_bih = E_m *W_ih .* sum((E_mi).* ActiveF_diff (u_hat,2).* ActiveF_diff (h,2));
        m_bih = beta1_bih * m_bih + (1-beta1_bih)*norm(DE_bih);
        v_bih = beta2_bih * v_bih + (1-beta2_bih)*norm(DE_bih).^2;
        alpha_t_bih = alpha_bih* (sqrt(1-beta2_bih^(kk)))/(1-beta1_bih^(kk));
        b_ih_new = b_ih - alpha_t_bih* m_bih./(sqrt(v_bih)+regular_coeff);
%         b_ih_new = b_ih - Lr*norm(DE_bih); %%% new bias (updated value) %% Stochastic gradeint descent
        
        DE_Whh = E_m *W_ih .* sum((E_mi).* ActiveF_diff (u_hat,2).* ActiveF_diff (h,2) .* h);
        m_hh = beta1_hh * m_hh + (1-beta1_hh)*DE_Whh;
        v_hh = beta2_hh * v_hh + (1-beta2_hh)*DE_Whh.^2;
        alpha_t_hh = alpha_hh* (sqrt(1-beta2_hh^(kk)))/(1-beta1_hh^(kk));
        W_hh_new =  W_hh- alpha_t_hh* m_hh./(sqrt(v_hh)+regular_coeff);
%         W_hh_new = W_hh - Lr*abs(DE_Whh); %%% new weight (updated value) %% Stochastic gradeint descent
        
        for k=2:w+1
            h(k,1) = ActiveF_diff(W_ih_new(k-1,1) * I_traning_set(k-1,1) + W_hh_new(k-1,1) * h(k-1,1) + b_ih_new,1);
            
        end
        h=h(2:end,1);
        u_hat= ActiveF_diff(W_hu_new .* h + b_hu_new,1);
        C_mi  = u-u_hat;
        DC_Wuu = sum(C_mi.* ActiveF_diff (u,2) .* u);

        m_uu = beta1_uu * m_uu + (1-beta1_uu)*DC_Wuu;
        v_uu = beta2_uu * v_uu + (1-beta2_uu)*DC_Wuu.^2;
        alpha_t_uu = alpha_uu* (sqrt(1-beta2_uu^(kk)))/(1-beta1_uu^(kk));
        W_uu_new =  W_uu - alpha_t_uu* m_uu/(sqrt(v_uu)+regular_coeff);
%        W_uu_new = W_uu - Lr* abs (DC_Wuu); %%% new weight (updated value) %% Stochastic gradeint descent

        DC_buu = sum(C_mi .* ActiveF_diff(u,2));
        m_buu = beta1_buu * m_buu + (1-beta1_buu)*DC_buu;
        v_buu = beta2_buu * v_buu + (1-beta2_buu)*DC_buu.^2;
        alpha_t_buu = alpha_buu* (sqrt(1-beta2_buu^(kk)))/(1-beta1_buu^(kk));
        b_uu_new = b_uu - alpha_t_buu* m_buu/(sqrt(v_buu)+regular_coeff);
%         b_uu_new = b_uu - Lr* abs(DC_buu); %%% new bias (updated value) %% Stochastic gradeint descent
%%        
        epi = norm(W_uu_new - W_uu) + norm(b_hu_new - b_hu) + norm(W_hu_new - W_hu) + norm(b_hu_new - b_hu);
        y1 = h; 
         for k =2:w
        y1 (k,1) = log(1+ exp( W_hh_new(k-1,1) * y1(k-1,1) + W_ih_new(k,1) * I_traning_set(k,1)+ b_ih_new ));
         end
%             y1 = log(1+exp(W_ih_new .* I_traning_set + b_ih_new + W_hh_new .* h));
            y2 = log(1+exp(W_uu_new .* u + b_uu_new));
            y3 = log(1+exp(W_hu_new .* h + b_hu_new));
 
        energ(i,kk) = abs(0.5 * norm((I_traning_set-u_hat)).^2 - sum(y1)); %%%
%         partition_f = 1;
%         e = exp(-(energ))/partition_f;
        grad_DE(i,kk) =norm(( sum(u-u_hat)/norm(u-u_hat)/2-(W_uu_new.*exp(W_uu_new.*u + b_uu_new))/(1+exp(W_uu_new.*u + b_uu_new))/log(10)).^2);
        energ2(i,kk)= exp(-1*sum(b_hu_new.*u_hat + u_hat.*W_hu_new.*h + b_uu_new.*u + u.*W_uu_new .*u));
        energ3(i,kk) = exp (0.5*sum((u-u_hat).^2));
        energ4 (i,kk) = norm(h .* W_ih_new .* I_traning_set + b_hu_new * h + b_ih_new * I_traning_set); %%% 
%        crossE_reconErr(i,kk) = -sum(u.*log(u_hat) + (1-u).*log(1-u_hat)); %%% cross-entropy of the reconstruction error (If the input is interpreted as either bit vectors or vectors of bit probabilities, cross-entropy of the reconstruction can be used)
        %% stopping criteria_1
%         if kk>10
%             if (energ3(i,kk-1)-energ3(i,kk-2)) < 0 
%                 if (energ3(i,kk)-energ3(i,kk-1)) > 0 
%                     if energ3(i,kk-1)< min(energ3(i,2:kk)) 
%                         flag_Learning=0;
%                     end
%                 elseif abs(energ3(i,kk)-energ3(i,kk-1)) < 10^(-10)
%                     count_dec = count_dec+1;
%                 end
%             else
%                 flag_Learning=1;
%             end
%         end
%     if kk>30000 || count_dec==10
%         flag_Learning=0;
%     end
     kk= kk+1;       
        %%  update weigth for next step  
        W_hu = W_hu_new;
        b_hu = b_hu_new;
        W_ih = W_ih_new;
        b_ih = b_ih_new;
        W_hh = W_hh_new;
        W_uu = W_uu_new;
        b_uu = b_uu_new;
        
    counter=counter+1;
    count_decF(i)=count_dec;
 
    %% stopping criteria_2 : cross validation
%     if k== k1*5
%        for k=2:w+1
%             u(k,1) = ActiveF_diff(W_uu(k-1,:)* u(k-1,1) + b_uu,1);
%             h(k,1) = ActiveF_diff(W_ih(k-1,:)* I_testing_set(k-1,1) + W_hh(k-1,:)* h(k-1,1) + b_ih,1);
%         end
%         u = u(2:end,1);
%         h = h(2:end,1);
%         u_hat = ActiveF_diff(W_hu .* h + b_hu,1); %%%logsig(h_i); %% output from neuron of hidden layer
%         E_testing(k1+1,1) = exp(0.5* sum((u-u_hat).^2));
%         E_testing_min = min(E_testing(2:k1+1));
%         GL = 100*(E_testing(k1+1,1)/E_testing_min-1); %%% Generalization Loss
%         if GL >1 
%           flag_Learning =0;
%         end
%         k1=k1+1;
%     end
    
    end
    x_est(i,1) = u(end);
% c_max =2; c_min=1;
% coef_out =c_max-rand(1)*c_min;
score(i,1)=mean(energ(i,2:end));
energ_timeW (i,1) = mean (energ(i,2:end));
% if i>1 
%     if score(i)< coef_out*median(score(1:i-1,1))
%     x_out(i)=x(i+w-1);
%     else
%     x_out(i)=x(i+w-2);
%     end
% else
%     x_out(i)=x(i+w-1);
% end
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
    
    count(i,1)=counter;
end
toc
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



