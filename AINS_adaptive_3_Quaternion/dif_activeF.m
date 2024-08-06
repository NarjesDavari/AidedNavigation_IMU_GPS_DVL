


function F=ActiveF_diff(x,type)

if type ==1
    F=tanh(x);
%     F= logsig (x);
elseif type==2
    F = 1-(tanh(x)).^2;
% D_activeF=exp(-x)./(1+exp(-x)).^2; %%%% diffrential of logsig
end