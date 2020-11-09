% This script shows the result for the accelerometer influence. It works  
% when x_a = 0ft. The same procedure as the one shown here was used for the
% remaining values of x_a
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

%% Linearization of the Model for altitude=15000ft and speed=500ft/s

disp('--- ANALYSIS OF THE INFLUENCE OF THE ACCELEROEMTER POSITION ---');
fprintf('Trimming model for altitude = 15000ft and speed = 500ft/s ... \n\n');

addpath obsmutoolsfornewermatlabversions -END % required for some new MATLAB versions
global fi_flag_Simulink

altitude = 15000;
velocity = 500;

% Initial guess for trim
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

% Find trim for model at desired altitude and velocity
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

% Find the state space model for the hifi model at the desired alt and vel.
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
[A_lo,B_lo,C_lo,D_lo] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);...
		dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

% Make state space model and MATLAB Matrix
SS_lo = ss(A_lo,B_lo,C_lo,D_lo);
mat_lo = [A_lo B_lo; C_lo D_lo];

input('\n\nPress a KEY to see the plot results. ');

%% Computation of the Reduced Matrices
% Here, only the variables that influence the normal acceleration are used.
% The output ins simply the normal acceleration (row 19 of matric C_lo)
A = A_lo([3 5 7 8 11 14], [3 5 7 8 11 14]);
B = B_lo([3 5 7 8 11 14],2);         
C = C_lo(19,[3 5 7 8 11 14]);
D = D_lo(19,2);

%% Computation of the Transfer Function, where the output is the normal acceleration
[b,a] = ss2tf(A,B,C,D);
F = tf(b,a);

%% Plot of the Time response to a negative step input of the elevator

%fig = figure();
t = 0: 0.05: 100;
%subplot(1,2,1);

% a negative step input response of a system is the same as a positive step 
% input response of the simmetric system
step(t,-F);
% ------

xlabel('Time');
ylabel('Normal Acceleration');
title('Time Response for a_n');

%% Detailed Plot of the Time response to a negative step input of the elevator

t = 0: 0.01: 1;
%subplot(1,2,2);

% a negative step input response of a system is the same as a positive step 
% input response of the simmetric system
stepplot(t,-F, 'b');
% ------

xlabel('Time');
ylabel('Normal Acceleration');
title('Time Response for a_n with x_a = 0ft');
grid on;
ylim([-0.05, 0.35]);
%set(fig, 'Position', [200 100 1100 500]);

%% Zeros Location for the Transfer Function

Z = zero(F);

clc;
fprintf('The zeros of this transfer function are located in:\n');
fprintf('%.4f   %.4f   %.4f   %.4f   %.4f\n', Z(1), Z(2), Z(3), Z(4), Z(5));