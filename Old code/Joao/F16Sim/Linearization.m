% This script runs the linearization of the proposed flight condition, and
% the state-space model is also reduced, so that the matrices only included
% the relevant variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

%% Linearization of the Model for altitude = 15000ft and speed = 500ft/s

disp('--- LINEARIZATION AND SYSTEM REDUCTION ---');
fprintf('Trimming model for altitude = 20000ft and speed = 600ft/s ... \n\n');

addpath obsmutoolsfornewermatlabversions -END % required for some new MATLAB versions
global fi_flag_Simulink

% Trim aircraft to desired altitude and velocity
altitude = 20000;
velocity = 600;

% Initial guess for trim
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

% Find trim for Hifi model at desired altitude and velocity
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

% Find the state space model for the hifi model at the desired alt and vel.
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
[A_lo,B_lo,C_lo,D_lo] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);...
		dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

% Make state space model and MATLAB matrix
SS_lo = ss(A_lo,B_lo,C_lo,D_lo);
mat_lo = [A_lo B_lo; C_lo D_lo];

% Selection of Components for the longitudinal and lateral matrices
A_longitude_lo = mat_lo([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);
B_longitude_lo = mat_lo([3 5 7 8 11 13 14], [19 20]);
C_longitude_lo = mat_lo([21 23 25 26 29], [3 5 7 8 11 13 14]);
D_longitude_lo = mat_lo([21 23 25 26 29], [19 20]);
SS_long_lo = ss(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo);

A_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [4 6 7 9 10 12 13 15 16]);
B_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [19 21 22]);
C_lateral_lo = mat_lo([22 24 25 27 28 30], [4 6 7 9 10 12 13 15 16]);
D_lateral_lo = mat_lo([22 24 25 27 28 30], [19 21 22]);
SS_lat_lo = ss(A_lateral_lo, B_lateral_lo, C_lateral_lo, D_lateral_lo);

input('\n\nPress a KEY to see eigenmotions characteristics. ');

%% Reduced LONGITUDINAL matrices obtained after linearizarion

A_long = A_longitude_lo([3,4,2,5,7,6],[3,4,2,5,7,6]);
C_long = C_longitude_lo([3,4,2,5],[3,4,2,5]);
D_long = D_longitude_lo([3,4,2,4],2);

% Creation of Matrix B by using a part of matrix A. Also, reduction of
% matrix A by taking out the elevator deflection and ignoring thrust
B_long = A_long([1,2,3,4],5);
A_long = A_long([1,2,3,4],[1,2,3,4]);

%% Reduced LATERAL matrices obtained from the FindF16Dynamics.m file
A_lat = A_lateral_lo([4,1,5,6,8,9],[4,1,5,6,8,9]);
C_lat = C_lateral_lo([4,1,5,6],[4,1,5,6]);
D_lat = D_lateral_lo([4,1,5,6],[2,3]);

% Creation of Matrix B by using a part of matrix A. Also, reduction of
% matrix A by taking out the aileron and rudder deflections
B_lat = A_lat([1, 2, 3, 4],[5,6]);
A_lat = A_lat([1,2,3,4],[1,2,3,4]);

%% Computation of eigenvalues for longitudinal and lateral modes

[vector_long,eigen_long] = eig(A_long);
[vctor_lat,eigen_lat] = eig(A_lat);

% Assignment of the eigenvalues to the corresponding eigenmotion

%LONGITUDINAL
if (eigen_long(1,1) < eigen_long(3,3))
    short_period(1) = eigen_long(1,1);
    phugoid(1) = eigen_long(3,3);
else
    short_period(1) = eigen_long(3,3);
    phugoid(1) = eigen_long(1,1);
end

% LATERAL
n=1;
m=1;

for i = 1 : 4
    if (imag(eigen_lat(i,i)) ~= 0)
        dutch_roll(n) = eigen_lat(i,i);
        n=n+1;
    else
        aux(m) = eigen_lat(i,i);
        m=m+1;
    end
end

if aux(1) < aux(2)
    roll = aux(1);
    spiral = aux(2);
else
    roll = aux(2);
    spiral = aux(1);
end

%% Computation of natural frequency (2), damping ratio (3), period (4) and time to damp to half amplitude (5) for the PERIODIC EIGENMOTIONS

short_period(2) = sqrt(real(short_period(1))^2 + imag(short_period(1))^2);
short_period(3) = -real(short_period(1))/short_period(2);
short_period(4) = (2*pi) / (abs(imag(short_period(1))));
short_period(5) = log(2) / (abs(real(short_period(1))));

phugoid(2) = sqrt(real(phugoid(1))^2 + imag(phugoid(1))^2);
phugoid(3) = -real(phugoid(1))/phugoid(2);
phugoid(4) = (2*pi) / (abs(imag(phugoid(1))));
phugoid(5) = log(2) / (abs(real(phugoid(1))));

dutch_roll(2) = sqrt(real(dutch_roll(1))^2 + imag(dutch_roll(1))^2);
dutch_roll(3) = -real(dutch_roll(1))/dutch_roll(2);
dutch_roll(4) = (2*pi) / (abs(imag(dutch_roll(1))));
dutch_roll(5) = log(2) / (abs(real(dutch_roll(1))));

%% Computation of natural frequency (2), time constant (3) and time to damp to half amplitude (4) for the ANTI-PERIODIC EIGENMOTIONS

roll(2) = abs(roll(1));
roll(3) = -1/roll(1);
roll(4) = log(2) / (abs(roll(1)));

spiral(2) = abs(spiral(1));
spiral(3) = -1/spiral(1);
spiral(4) = log(2) / (abs(spiral(1)));

%% Display of Eigenmotion Values
clc

fprintf('Eigenvalues of Short Period: %.4f +/- %.4fi \n', real(short_period(1)), imag(short_period(1)));
fprintf('Natural Freq.: %.4frad/s   Damp. Ratio: %.4f   Period: %.4fs   Time to Half Amp.: %.4fs \n\n', short_period(2),short_period(3),short_period(4),short_period(5));
fprintf('Eigenvalues of Phugoid: %.4f +/- %.4fi \n', real(phugoid(1)), imag(phugoid(1)));
fprintf('Natural Freq.: %.4frad/s   Damp. Ratio: %.4f   Period: %.4fs   Time to Half Amp.: %.4fs \n\n', phugoid(2),phugoid(3),phugoid(4),phugoid(5));
fprintf('Eigenvalues of Dutch Roll: %.4f +/- %.4fi \n', real(dutch_roll(1)), imag(dutch_roll(1)));
fprintf('Natural Freq.: %.4frad/s   Damp. Ratio: %.4f   Period: %.4fs   Time to Half Amp.: %.4fs \n\n', dutch_roll(2),dutch_roll(3),dutch_roll(4),dutch_roll(5));

fprintf('Eigenvalue of Roll: %.4f \n', roll(1));
fprintf('Natural Freq.: %.4frad/s   Time Const.: %.4fs   Time to Half Amp.: %.4fs \n\n', roll(2),roll(3),roll(4));
fprintf('Eigenvalue of Spiral: %.4f \n', spiral(1));
fprintf('Natural Freq.: %.4frad/s   Time Const.: %.4fs   Time to Half Amp.: %.4fs \n\n', spiral(2),spiral(3),spiral(4));

%% Time Responses

% Short Period

%Reduced System Matrices
A_aux = A_long([2,4],[2,4]);
B_aux = B_long([2,4],1);
C_aux = C_long([2,4],[2,4]);
D_aux = D_long([2,4],1);

[b,a] = ss2tf(A_aux, B_aux, C_aux, D_aux);

F = tf(b(1,:),a);
G = tf(b(2,:),a);
t = 0:0.01:10;
s = tf('s');

figure;
subplot(2,1,1);
step(t,F);
xlabel('time');
ylabel('angle of attack');
title('Short Period Response');

subplot(2,1,2);
step(t,G);
xlabel('time');
ylabel('pitch rate');
title('Short Period Response');

%Phugoid

%Reduced System Matrices
A_aux = A_long([1,3],[1,3]);
B_aux = B_long([1,3],1);
C_aux = C_long([1,3],[1,3]);
D_aux = D_long([1,3],1);

[b,a] = ss2tf(A_aux, B_aux, C_aux, D_aux);

F = tf(b(1,:),a);
G = tf(b(2,:),a);
t = 0:0.01:10;
s = tf('s');

figure;
subplot(2,1,1);
step(t,F);
xlabel('time');
ylabel('speed');
title('Phugoid Response');

subplot(2,1,2);
step(t,G);
xlabel('time');
ylabel('pitch angle');
title('Phugoid Response');

%Dutch Roll

%Reduced System Matrices
A_aux = A_lat([1,3],[1,3]);
B_aux = B_lat([1,3],1);
C_aux = C_lat([1,3],[1,3]);
D_aux = D_lat([1,3],1);

[b,a] = ss2tf(A_aux, B_aux, C_aux, D_aux);

F = tf(b(1,:),a);
G = tf(b(2,:),a);
t = 0:0.01:10;
s = tf('s');

figure;
subplot(2,1,1);
step(t,F);
xlabel('time');
ylabel('speed');
title('Phugoid Response');

subplot(2,1,2);
step(t,G);
xlabel('time');
ylabel('pitch angle');
title('Phugoid Response');


%% Gibson Criterion

