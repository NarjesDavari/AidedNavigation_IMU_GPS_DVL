

%%%% diffrential of logsig
function Dlogsig=dif_activeF(x)


Dlogsig=exp(-x)./(1+exp(-x));
end