
%% for claculation of lower and upper limitation (-2*sigma,+2*sigma) in estimated value of variational bayesian
vb_b = beta;
vb_a = alpha;
 theta = (0.05:0.01:2);
    p = -(vb_a+1)*log(theta) - vb_b./theta;
    p = p - median(p);
    p = exp(p);
    p = p./sum(p);

    c = cumsum(p);
    akf_Q_05(k) = theta(max([find(c < 0.05) 1]));
    akf_Q_95(k) = theta(min([find(c > 0.95) length(c)]));