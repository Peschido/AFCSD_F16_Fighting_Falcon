% clear workspace
clear all
close all
clc

%% 
disp('PITCH RATE COMMAND SYSTEM');

% load data from chapter 6
load CH6Matrices.mat   

%% Pitch Rate Response to a step input for the 4- and 2-state model

% A_sp, B_sp, C_sp and D_sp => simplified 2-state model
A_sp = A_long([2,4],[2,4]); 
B_sp = B_long([2,4],1);
C_sp = C_long([2,4],[2,4]);
D_sp = D_long([2,4],1);

% to get the transfer function of the pitch rate to an elevator input in
% the 4-state model, use 'F = tf(a(4,:),b)' because q is the 4th state in
% the model. The model contains two inputs but only elevator is considered,
% hence B_long(:,1) and D_long(:,1) are used.
[a,b] = ss2tf(A_long,B_long(:,1),C_long,D_long(:,1));
tf_4s = tf(a(4,:),b);

% in the 2 state model, q is the 2nd state
[c,d] = ss2tf(A_sp,B_sp(:,1),C_sp,D_sp(:,1));
tf_2s = tf(c(2,:),d);

% Print transfer functions
tf_4s
tf_2s

% Plot of the pitch rate response to a step deflection in the elevator
t = 0:1:1000; % time range

figure(1);
step(tf_4s,t);
hold on % plot in the same figure
step(tf_2s);

% formatting
ylabel('q [rad/s]');
title('Pitch Rate Response to Step Input');
legend('4 state', '2 state');
hold off % stop plotting in the same figure

% Initial response of the time response of q (pitch rate) to a step deflection 
% in the elevator (delta_e) for both the 2-state and 4-state models (short
% period)
T = 0:0.05:15;
figure(2); % new figure
step(tf_4s, t);
hold on % plot in the same figure
step(tf_2s, t);

ylabel('q [rad/s]');
title('Initial Response - Pitch Rate Response to a Step Input');
legend('4 state', '2 state');
hold off % stop plotting in the same figure


%% Procedure to obtain the pitch damper
% Desired Values 
w_n = 5.49; % desired natural frequency
damp_ratio = 0.5; % desired damping ratio

% p1, p2 are the desired poles
p1 = -w_n*damp_ratio + w_n*sqrt(damp_ratio^2 -1)/2
p2 = -w_n*damp_ratio - w_n*sqrt(damp_ratio^2 -1)/2


% the 'place' functions computes the gain such that the system has poles in
% p1 and p2
poles = [p1 p2];
K = place(A_sp,B_sp, poles) % K_alpha, K_q

% Verification, gust case
alpha = atan(4.572/182.88) ; % rad
elevator = alpha * K(1)


%% evaluation of CAP and Gibson criteria

% Transfer function (derived in the report)
s = tf('s');
TF = (7.322*(4.115 + s)) / (s^2 + 5.49*s + 30.14);

t = 0:0.05:10;

% Pitch rate step response
Y = step(TF,t);
figure(3);
step(TF, t);
ylabel('q [rad/s]');
title('Pitch rate response to a step input');

% q_m = maximum of the step response, q_s = 1 because q stabilizes at 1
q_m = max(Y)

% Ramp input 
S = size(t);
S = S(2);

ramp = 0:0.05:(t(S)/2);
steady = t((S+1)/2) * ones(1,(S-1)/2);
input = [ramp steady];

% Pitch attitude ramp response
[y,t]=lsim(TF,input,t);
figure(4);
plot(t,y,'r');
hold on
plot(t,input,'b');
ylabel('theta [rad]');
xlabel('Time (seconds)');
title('Pitch angle response to a ramp input');
legend('pitch angle','ramp input');
hold off

% DBfinal is the final value of the repsonse (steady)
% DBmax is the maximum value of the pitch rate response
DBfinal = y(S) 
DBmax = max(y)















