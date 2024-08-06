
function kalman_filter=kalman_filter(xn,xe,h)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% xn=xn(:,2);
% xe=xe(:,2);
% h=h(:,2);

Y=[xn(:,2) xe(:,2) h(:,2)];
Time=h(:,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lat=input.Path.P_geo(:,1);
% lon=input.Path.P_geo(:,2);
% alt=input.Path.P_geo(:,3);

% Y=[lat lon alt];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Transition matrix for the continous-time system.
F = [0 0 0 1 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0];

% Noise effect matrix for the continous-time system.
L = [0 0 0;
     0 0 0;
     0 0 0;
     0 0 0;
     0 0 0;
     0 0 0;
     1 0 0;
     0 1 0;
     0 0 1];

% Stepsize
dt = 0.2;

% Process noise variance
% q1 = 10000;
% q2 = 10000;
% q3 = 40;
q1 = 1;
q2 = 1;
q3 = 1;
Qc = diag([q1 q2 q3]);

% Discretization of the continous-time system.
[A,Q] = lti_disc(F,L,Qc,dt);

% Measurement model.
H = [1 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0];

% Variance in the measurements.
% r1 = 100;
% r2 = 100;
% r3 = 10500;
r1 = 1e3;
r2 = 1e2;
r3 = 1e21;
R  = diag([r1 r2 r3]);

% Generate the data.
Dlength = length(Y);

% Initial guesses for the state mean and covariance.
m = [Y(1,1) Y(1,2) Y(1,3) 0 0 0 0 0 0]';
P = Q;

%% Space for the estimates.
MM = zeros(Dlength,size(m,1));
PP = zeros(size(m,1), size(m,1), Dlength);


% Filtering steps.
for i = 1:size(Y,1)
   [m,P] = kf_predict(m,P,A,Q);
   [m,P] = kf_update(m,P,Y(i,:)',H,R);
   MM(i,:) = m;
   PP(:,:,i) = P;
end
% Smoothing step(Backward Filter).
[SM,SP] = rts_smooth(MM,PP,A,Q);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kalman_filter.xn_f=MM(:,1);
kalman_filter.xe_f=MM(:,2);
kalman_filter.h_f=MM(:,3);
kalman_filter.h_f(:,2)=Time;
% 
kalman_filter.xn_s=SM(:,1);
kalman_filter.xe_s=SM(:,2);
kalman_filter.h_s=SM(:,3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot(kalman_filter.xn_s,'r')
hold on
plot(xn(:,2))
figure
plot(kalman_filter.xe_s,'r')
hold on
plot(xe(:,2))
figure
plot(kalman_filter.h_s,'r')
hold on
plot(h(:,2))

