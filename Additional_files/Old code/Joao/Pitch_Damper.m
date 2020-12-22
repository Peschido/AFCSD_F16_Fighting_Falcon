clear all
close all
clc

%% Data Loading
disp('--- PITCH RATE COMMAND SYSTEM DESIGN ---');

load pitch_damper

input('\n\nPress ENTER to see pitch rate response to a step input. ');
clc

%% Pitch Rate Response to a step input, for both the 2-state and 4-state model

% A_sp, B_sp, C_sp and D_sp are the reduced matrices of the short period
% reduced model, obtained from the 4-state model which was, in turn, also
% reduced in the 'Linearization' script
A_sp = A_long([2,4],[2,4]);
B_sp = B_long([2,4],1);
C_sp = C_long([2,4],[2,4]);
D_sp = D_long([2,4],1);

% Obtainment of the transfer Function 'q(s) / delta_e(s)' for the 4-state 
% model. q (pitch rate) is the 4th state in this model, hence the
% command: 'F = tf(a(4,:),b)'. Only the elevator (delta_e) is considered,
% that is why the matrices B_long(:,1) and D_long(:,1) are used
[a,b] = ss2tf(A_long,B_long(:,1),C_long,D_long(:,1));
TF_4state = tf(a(4,:),b);

% Obtainment of the transfer Function 'q(s) / delta_e(s)' for the 2-state 
% model. q (pitch rate) is the 2th state in this model, hence the
% command: 'G = tf(c(2,:),d)'. Only the elevator (delta_e) is considered,
% that is why the matrices B_sp(:,1) and D_sp(:,1) are used
[c,d] = ss2tf(A_sp,B_sp(:,1),C_sp,D_sp(:,1));
TF_2state = tf(c(2,:),d);

%Display of both transfer functions
disp('Transfer Function of the 4-state model:');
TF_4state

disp('Transfer Function of the 2-state model:');
TF_2state

% Plot of the time response of q (pitch rate) to a step deflection in the 
% elevator (delta_e) for both the 2-state and 4-state models
T = 0:0.5:1000;
figure(1);
step(TF_4state,T);
hold on
step(TF_2state,T);

ylabel('q [rad/s]');
title('Pitch Rate Response to a Step Input');
legend('4 state', '2 state');
hold off

% Deitaled view of the time response of q (pitch rate) to a step deflection 
% in the elevator (delta_e) for both the 2-state and 4-state models
T = 0:0.05:20;
figure(2);
step(TF_4state,T);
hold on
step(TF_2state,T);

ylabel('q [rad/s]');
title('Detailed view - Pitch Rate Response to a Step Input');
legend('4 state', '2 state');
hold off

input('Press ENTER to see the pole placement for the pitch damper.');
clc

%% Design of the Pitch Rate Command - Pole Placement
% Desired Values for the natural frequency (omega_n) and for the damping
% ratio (damp_ratio)
omega_n = 5.49;
damp_ratio = 0.5;

% p1 and p2 are the locations of the desired poles
p1 = -omega_n*damp_ratio + omega_n*sqrt(damp_ratio^2 -1)/2;
p2 = -omega_n*damp_ratio - omega_n*sqrt(damp_ratio^2 -1)/2;

fprintf('To get a natural frequency of %.2f rad/s and a damping ratio of %.1f,\n',omega_n, damp_ratio);
fprintf('the poles should be located in %.3f+/-%.3fi.\n\n', real(p1), imag(p1));

% For a given system defined by the matrices 'A_sp' and 'B_sp', the command 
% 'place' computes the gain K that leads the system to have a pole in p1
% and p2
P_sp = [p1 p2];
K = place(A_sp,B_sp,P_sp);

fprintf('The gain matrix is given by:\n');
fprintf('K_alpha = %.3f      K_q = %.3f\n\n', K(1), K(2));

input('Press ENTER to see the plots that help determine the Gibson/CAP Criteria.');
clc

%% CAP and Gibson criteria verification: plots

% Definition of the final Transfer Functionm including the closed loop
% system (obtained with the pole placement technique) and the prefilter.
% The derivation of this transfer function is explained in Chapter 4,
% Section 4.3
s = tf('s');
F = (7.322*(4.115 + s)) / (s^2 + 5.49*s + 30.14);

T = 0:0.05:10;

% Pitch rate step response
Y = step(F,T);
figure(3);
step(F,T);
ylabel('q [rad/s]');
title('Pitch rate response to a step input');

% q_m is defined as the maximum value of the pitch rate response to a step
% q_s is the final value of the same response. Because the input is a step
% and the pitch rate follows it with no error, q_s = 1
q_m = max(Y);

fprintf('Values needed for the Gibson Dropback criterion:\n');
fprintf('q_m = %.3f     q_s = 1\n\n', q_m);

% Computation of the ramp input, used in the time response for the pitch
% attitude
S = size(T);
size = S(2);

ramp = 0:0.05:(T(size)/2);
final = T((size+1)/2) * ones(1,(size-1)/2);
input = [ramp final];

% Pitch attitude ramp response
[y,t]=lsim(F,input,T);
figure(4);
plot(t,y,'r');
hold on
plot(t,input,'b');
ylabel('theta [rad]');
xlabel('Time (seconds)');
title('Pitch angle response to a ramp input');
legend('pitch angle','ramp input');
hold off

% DB_max is the maximum value of the pitch attitude ramp response. DB_final
% is the final value of the same response. Because the pitch attitude angle
% follows the input with no error when it gets constant, DB_final can be
% the last value of the input
DB_final = input(size);
DB_max = max(y);

fprintf('Values obtained from the pitch attitude ramp response:\n');
fprintf('DB_max = %.3f     DB_final = %.3f\n\n', DB_max, DB_final);

fprintf(' --- END OF PROGRAM ---\n');