


function F=ActiveF_diff(x,type)

if type ==1
    F=tanh(x);
%     F= logsig (x);
%     F= x ./ (1+abs(x));
% F = x;
% F=log(1+exp(x));

elseif type==2
    F = 1-(tanh(x)).^2;
% F=exp(-x)./(1+exp(-x)).^2; %%%% diffrential of logsig 
% F = ones(length(x),1);
% F = exp(x)./(1+exp(x));
%     for i=1:length(x)
%         if x(i)>0
%             F(i,1) = 1/(1+x(i))^2;
%         else
%             F(i,1) = -1/(1+x(i))^2;
%         end
%     end
end