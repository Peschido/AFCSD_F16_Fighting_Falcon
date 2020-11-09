% This script runs the linearization of the proposed flight condition, and
% the state-space model is also reduced, so that the matrices only included
% the relevant variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

makeplots = false;
fig = 1

%% Linearization of the Model for altitude = 20000ft and speed = 600ft/s

disp('--- LINEARIZATION AND SYSTEM REDUCTION ---');
fprintf('Trimming model for altitude = 20000ft and speed = 600ft/s ... \n\n');

addpath obsmutoolsfornewermatlabversions -END % required for some new MATLAB versions
global fi_flag_Simulink

% Trim aircraft to desired altitude and velocity
altitude = 10000;
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
% This is the first matrix
A_long = A_longitude_lo([3,4,2,5,7,6],[3,4,2,5,7,6])
C_long = C_longitude_lo([3,4,2,5],[3,4,2,5])
D_long = D_longitude_lo([3,4,2,4],2)

% Creation of Matrix B by using a part of matrix A. Also, reduction of
% matrix A by taking out the elevator deflection and ignoring thrust
B_long = A_long([1,2,3,4],5)
A_long = A_long([1,2,3,4],[1,2,3,4])

%% Reduced LATERAL matrices obtained from the FindF16Dynamics.m file
A_lat = A_lateral_lo([4,1,5,6,8,9],[4,1,5,6,8,9])
C_lat = C_lateral_lo([4,1,5,6],[4,1,5,6])
D_lat = D_lateral_lo([4,1,5,6],[2,3])

% Creation of Matrix B by using a part of matrix A. Also, reduction of
% matrix A by taking out the aileron and rudder deflections
B_lat = A_lat([1, 2, 3, 4],[5,6])
A_lat = A_lat([1,2,3,4],[1,2,3,4])

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



%% Computation of natural frequency (2), damping ratio (3), period (4) and time to damp to half amplitude (5) for the PERIODIC EIGENMOTIONS

short_period(2) = sqrt(real(short_period(1))^2 + imag(short_period(1))^2);
short_period(3) = -real(short_period(1))/short_period(2);
short_period(4) = (2*pi) / (abs(imag(short_period(1))));
short_period(5) = log(2) / (abs(real(short_period(1))));


A_lon2 = A_long([2,4],[2,4])
B_lon2 = B_long([2,4])
C_lon2 = C_long([2,4],[2,4])
D_lon2 = D_long([2,4])


%% Time Responses
SS_lon2 = ss(A_lon2, B_lon2, C_lon2, D_lon2);
A_long = A_long([2,4,1,3],[2,4,1,3]);
B_long = B_long([2,4,1,3]);
C_long = C_long([2,4,1,3],[2,4,1,3]);
D_long = D_long([2,4,1,3]);
SS_lon_lofi = ss(A_long, B_long, C_long, D_long);

eig_lon = eig(A_long)
eig_lon2= eig(A_lon2)

[eig_vec_lon, eig_val_lon] = eig(A_long); % 1&2.Short period 3&4.Phugoid
[eig_vec_lon2, eig_val_lon2] = eig(A_lon2);

xsp = eig_vec_lon(:,1)+eig_vec_lon(:,2);
xph = eig_vec_lon(:,3)+eig_vec_lon(:,4);
xsp2 = eig_vec_lon2(:,1)+eig_vec_lon2(:,2);
xph2 = eig_vec_lon2(:,1)+eig_vec_lon2(:,2);

list_motions_lon = ["Short Period Time Response", "Phugoid Time Response"];

% Transfer functions of the longitudinal system dynamics
Hlon    = tf(SS_lon_lofi);
Hlon2    = tf(SS_lon2);
plon    = pole(Hlon);
plon2    = pole(Hlon2);

% Create time vector(s)
time_sp = 0:0.01:1000;
time_ph = 0:0.01:1000;

y_sp   = zeros(4, length(time_sp));
y_sp2   = zeros(2, length(time_sp));
y_ph   = zeros(4, length(time_ph));
y_ph2   = zeros(2, length(time_ph));


% Calculating eigenmotion responses based on eigenvectors xsp and xph
for i=1:length(time_sp)
    y_sp(:,i) = C_long*expm(time_sp(i)*A_long)*xsp;
    y_sp2(:,i) = C_lon2*expm(time_sp(i)*A_lon2)*xsp2; 
end

for i=1:length(time_ph)
   y_ph(:,i) = C_long*expm(time_ph(i)*A_long)*xph;
   y_ph2(:,i) = C_lon2*expm(time_ph(i)*A_lon2)*xph2; 
end

% Plotting Longitudinal Responses
y_labels    = ["Velocity [ft/s]", "\alpha [deg]", "\theta [deg]", "q [deg/s]"];
titles_sp   = ["\Delta V", "\Delta \alpha", "\Delta \theta", "\Delta q"];
titles_ph   = ["\Delta V", "\Delta \alpha", "\Delta \theta", "\Delta q"];
%%

figure(1)
plot(time_sp, y_sp2(2,:))
xlabel('Time [sec]')
ylabel(y_labels(4))
%title(titles_sp(j))
grid on;


%step(SS_lon2, SS_lon_lofi,1000)
%legend('2 States','4 States')

disp('4 state TF')
tf(SS_lon_lofi)
disp('2 state TF')
tf(SS_lon2)


% if makeplots == true
%     for i = 1:1 
%         figure(fig)
%         fig = fig+1;
%         for j = 1
%             if i == 1
%                 subplot(4,1,j)
%                 hold on
%                 plot(time_sp, y_sp(j,:))
%                 plot(time_sp, y_sp2(j,:))
%                 hold off
%                 xlabel('Time [sec]')
%                 ylabel(y_labels(j))
%                 %title(titles_sp(j))
%                 grid on;
%             else
%                 subplot(4,1,j)
%                 hold on
%                 plot(time_ph, y_ph(j,:))
%                 plot(time_ph, y_ph(j,:))
%                 hold off
%                 xlabel("Time [sec]")
%                 ylabel(y_labels(j))
%                 %title(titles_ph(j))
%                 grid on;
%             end
%                 %sgtitle(list_motions_lon(i));
%                 %saveas(gcf,list_motions_lon(i)+'.png');
%         end
%     end
% end

%Pole placement
p = [-2.745 + 2.377i, -2.745 - 2.377i];
K = place(A_lon2,B_lon2,p)