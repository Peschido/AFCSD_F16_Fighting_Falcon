% AE4301P Exercise Automatic Flight Control System Design
% Date: 11 February 2020
% Authors: Luis Gambetta, Bart Debeuckelaere, Gregory Sanders

%------------------------------------------------------------
% Code for Chapter 5 Trim and Linearization
%------------------------------------------------------------

% clear workspace
clear all
close all
clc
clear;

addpath obsmutoolsfornewermatlabversions -END % required for some new MATLAB versions

global fi_flag_Simulink

newline = sprintf('\n');


%% Trim aircraft to alt = 15000 ft and speed= 500 ft/s

altitude = 15000;   % ft
velocity = 500;     % ft/s

% change x_a manually in lin_f16block

%% Initial guess for trim

thrust = 5000;              % [lbs]
elevator = -0.09;           % [degrees]
alpha = 8.49;               % [degrees]
rudder = -0.01;             % [degrees]
aileron = 0.01;             % [degrees]

%% Trim Lofi model at chosen flight condition

disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;    % 0: Lofi, 1: Hifi
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

%% Find the state space model for the hifi model at the desired alt and vel.

trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
[A_lo,B_lo,C_lo,D_lo] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);...
		dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

%% State space model

SS_lo = ss(A_lo,B_lo,C_lo,D_lo);

%% Make MATLAB matrix
%%
mat_lo = [A_lo B_lo; C_lo D_lo];

%% Linearized Output Equation (y = Cx + Du) for normal accelleration a_n at cg (x_a = 0)
%%
% Take row 19 of matric C_lo

A = A_lo([3 5 7 8 11 14], [3 5 7 8 11 14]);
B = B_lo([3 5 7 8 11 14],2);         
C = C_lo(19,[3 5 7 8 11 14])
D = D_lo(19,2);

%% Determine the elevator-to-normal-acceleration transfer function
%%
[b,a] = ss2tf(A,B,C,D);     % a is numerator and b is denomenator
TF = tf(b,a)                % Transfer function

TFmin = minreal(TF)         % Reduced Transfer Function (pole-zero cancellation)

%% Plot the normal acceleration response to a negative step elevator command
%%
% t = 0: 0.05: 100;           % list of time points 0 to 100 sec.
% subplot(1,2,1);             % subplot 1
% step(t,-TF);                % negative step input
% 
% xlabel('Time');
% ylabel('Normal Acceleration');
% title('Time Response for a_n with x_a = 0ft');
% grid on;


t = 0: 0.01: 1;             % list of time points 0 to 1 sec.
%subplot(1,2,2);             % subplot 2
step(t,-TF);                % negative step input

xlabel('Time');
ylabel('Normal Acceleration');
title('Time Response for a_n with x_a = 15ft');
grid on;
ylim([-0.05, 0.35]);


%% Zeros of Transfer Function
%%
Z = zero(TF)

