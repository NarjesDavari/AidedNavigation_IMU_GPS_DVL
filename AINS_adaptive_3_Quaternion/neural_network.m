clear
tic
load('Real_Measuremet_test1_22th_2','Real_Measurement')
x=Real_Measurement.DVL(3570:3615,2);

l=4; %%number of units in input layer
k=1; %%number of units in output layer
m=1; %% number of hidden layer
n=4;%% number of units in hidden layer
w=l;
epsi=5e-2;
Lr=0.05;

% I=zeros(l,1); %% input to NN
h_i=zeros(n,1); %% input to hidden layer
h_o=zeros(n,1); %% output from hidden layer

% gama=?? %% learning rate
W_ih=zeros(l,n);
W_ho=zeros(n,k);

k_t=1;
for i=1:length(x)-w
error=1;
count=0;
flag=0;
W_ih=0.001*randn(l,n); %% weigth between input layer and hidden layer
DW_ih = zeros(l,n);
W_ho=0.001*randn(n,k); %% weigth between hidden layer and output layer
DW_ho = zeros(n,k);
I = x(i:w+i-1)';
 target= median(I)-k_t* abs(median(I(1:end-1)-I(2:end)));
    while flag==0
        count=count+1;
        
    %% Forward Propagation
        h_i=I*W_ih; %% input to neuron of hidden layer
        h_o= logsig(h_i); %%%tanh(h_i); %% output from neuron of hidden layer

        y_i=h_o*W_ho; %% input to last layer
        y_o= y_i;  %%%tanh(y_i); %%%logsig(y_i); %% output from neuron of output layer
        % [target,] = ErrorEval(y_o);
       
    %% Backward Propagation
        Dy_i=(target-y_o);%%% 2*(target-y_o).*dif_activeF(y_i);
        DW_ho=Dy_i* h_o;  %%% matrix nxk
        W_ho_new=W_ho'-Lr*abs(DW_ho); %%% matrix nxk  %% W_ho_new=W_ho- gama*(gradent_costFunction) %% Stochastic gradeint descent

        Dh_i=(Dy_i.*W_ho').*dif_activeF(h_i);
        for j=1:length(Dh_i)
        DW_ih(:,j)=(Dh_i(j).*I)'; %%%  matrix 1xn
        end
        W_ih_new=W_ih-Lr*abs(DW_ih); %%% matrix 1xn
        error = abs(target-y_o);
        if error < epsi
            flag=1;
        end
        W_ih = W_ih_new;
        W_ho = W_ho_new';
    end
yout(i+w,1)=y_o;
target_o (i+w,1)= target;
% x(i+w-1)=y_o;
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



