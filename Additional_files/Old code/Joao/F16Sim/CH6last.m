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

SS_lon_lofi = ss(A_long, B_long, C_long, D_long);

eig_lon = eig(A_long)
[eig_vec_lon, eig_val_lon] = eig(A_long); % 1&2.Short period 3&4.Phugoid

xsp = eig_vec_lon(:,1)+eig_vec_lon(:,2);
xph = eig_vec_lon(:,3)+eig_vec_lon(:,4);

list_motions_lon = ["Short Period Time Response", "Phugoid Time Response"];

% Transfer functions of the longitudinal system dynamics
Hlon    = tf(SS_lon_lofi);
plon    = pole(Hlon);

% Create time vector(s)
time_sp = 0:0.01:8;
time_ph = 0:0.01:1400;

y_sp   = zeros(4, length(time_sp));
y_ph   = zeros(4, length(time_ph));

% Calculating eigenmotion responses based on eigenvectors xsp and xph
for i=1:length(time_sp)
    y_sp(:,i) = C_long*expm(time_sp(i)*A_long)*xsp;  
end

for i=1:length(time_ph)
   y_ph(:,i) = C_long*expm(time_ph(i)*A_long)*xph; 
end

% Plotting Longitudinal Responses
y_labels    = ["Velocity [ft/s]", "\alpha [deg]", "\theta [deg]", "q [deg/s]"];
titles_sp   = ["\Delta V", "\Delta \alpha", "\Delta \theta", "\Delta q"];
titles_ph   = ["\Delta V", "\Delta \alpha", "\Delta \theta", "\Delta q"];

if makeplots == true
    for i = 1:2 
        figure(fig)
        fig = fig+1;
        for j = 1:4
            if i == 1
                subplot(4,1,j)
                plot(time_sp, y_sp(j,:))
                xlabel('Time [sec]')
                ylabel(y_labels(j))
                %title(titles_sp(j))
                grid on;
            else
                subplot(4,1,j)
                plot(time_ph, y_ph(j,:))
                xlabel("Time [sec]")
                ylabel(y_labels(j))
                %title(titles_ph(j))
                grid on;
            end
                %sgtitle(list_motions_lon(i));
                %saveas(gcf,list_motions_lon(i)+'.png');
        end
    end
end

SS_lat = ss(A_lat, B_lat, C_lat, D_lat);

eig_lat = eig(A_lat)
[eig_vec_lat, eig_val_lat] = eig(A_lat); % 3.Aperiodic Roll 1&2.Dutch Roll 4.Spiral Mode

xap = eig_vec_lat(:,3);
xdr = eig_vec_lat(:,1)+eig_vec_lat(:,2);
xsm = eig_vec_lat(:,4);

list_motions_lat = ["Aperiodic Roll Time Response", "Dutch Roll Time Response","Spiral Time Response"];

% Transfer functions of the longitudinal system dynamics
Hlat = tf(SS_lat); 
plat = pole(Hlat);

% Create time vector(s)
time_ap = 0:0.01:4.5;
time_dr = 0:0.01:20;
time_sm = 0:0.01:700;

y_ap = zeros(4, length(time_ap));
y_dr = zeros(4, length(time_dr));
y_sm = zeros(4, length(time_sm));

% Calculating eigenmotion responses based on eigenvectors xap, xdr and xsm
for i=1:length(time_ap)
    y_ap(:,i) = C_lat*expm(time_ap(i)*A_lat)*xap;
end

for i=1:length(time_dr)
    y_dr(:,i) = C_lat*expm(time_dr(i)*A_lat)*xdr;
end

for i=1:length(time_sm)
    y_sm(:,i) = C_lat*expm(time_sm(i)*A_lat)*xsm;
end

% Plotting Lateral responses
y_labels    = ["\beta [deg]", "\phi[deg]", "p [deg/s]", "r [deg/s]"];
titles_ap   = ["\Delta \beta", "\Delta \phi", "\Delta p", "\Delta r"];
titles_dr   = ["\Delta \beta", "\Delta \phi", "\Delta p", "\Delta r"];
titles_sm   = ["\Delta \beta", "\Delta \phi", "\Delta p", "\Delta r"];

if makeplots == true
    for i = 1:3 
        figure(fig)
        fig = fig+1;
        for j = 1:4
            if i == 1
                subplot(4,1,j)
                plot(time_ap, y_ap(j,:));
                xlabel('Time [sec]')
                ylabel(y_labels(j))
                %title(titles_ap(j))
                grid on;
            elseif i == 2
                subplot(4,1,j)
                plot(time_dr, y_dr(j,:));
                xlabel("Time [sec]")
                ylabel(y_labels(j))
                %title(titles_dr(j))
                grid on;
            else 
                subplot(4,1,j)
                plot(time_sm, y_sm(j,:));
                xlabel("Time [sec]")
                ylabel(y_labels(j))
                %title(titles_sm(j))
                grid on;
            end
                %sgtitle(list_motions_lat(i));
                %saveas(gcf,list_motions_lat(i)+'.png');
        end
    end
end

%% Chapter 7

ss(A_long, B_long, C_long,


