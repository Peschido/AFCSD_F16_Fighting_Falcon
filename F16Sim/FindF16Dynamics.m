%================================================
%     Matlab Script File used to linearize the 
%     non-linear F-16 model. The program will 
%     Extract the longitudal and lateral 
%     direction matrices.  These system matrices 
%     will be used to create pole-zero mapping
%     and the bode plots of each to each control
%     input.
% Author: Richard S. Russell
% 
%================================================
%% Producing Latex code of a matrix
%%
clear;

addpath obsmutoolsfornewermatlabversions -END % required for some new MATLAB versions

global fi_flag_Simulink

newline = sprintf('\n');

ho_value_on = false; 
plotting = false;
display_results = false
eigenmotion_plots = false
saving = false



%% Trim aircraft to desired altitude and velocity
%%
%altitude = input('Enter the altitude for the simulation (ft)  :  ');
%velocity = input('Enter the velocity for the simulation (ft/s):  ');
altitude = 5000
velocity = 300

%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

%% Find trim for Hifi model at desired altitude and velocity
%%
if ho_value_on
    disp('Trimming High Fidelity Model:');
    fi_flag_Simulink = 1;
    [trim_state_hi, trim_thrust_hi, trim_control_hi, dLEF, xu_hi] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

    %% Find the state space model for the hifi model at the desired alt and vel.
    %%
    trim_state_lin = trim_state_hi; trim_thrust_lin = trim_thrust_hi; trim_control_lin = trim_control_hi;
    [A_hi,B_hi,C_hi,D_hi] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3); ...
            dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);
end
%% Find trim for Hifi model at desired altitude and velocity
%%
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

%% Find the state space model for the hifi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
[A_lo,B_lo,C_lo,D_lo] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);...
		dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

%% Make state space model
%%
if ho_value_on
    SS_hi = ss(A_hi,B_hi,C_hi,D_hi);
end
SS_lo = ss(A_lo,B_lo,C_lo,D_lo);

%% Make MATLAB matrix
%%
if ho_value_on
    mat_hi = [A_hi B_hi; C_hi D_hi];
end
mat_lo = [A_lo B_lo; C_lo D_lo];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Directional %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Select the components that make up the longitude A matrix
%%
if ho_value_on
    A_longitude_hi = mat_hi([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);
end
A_longitude_lo = mat_lo([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);

%% Select the components that make up the longitude B matrix
%%
if ho_value_on
    B_longitude_hi = mat_hi([3 5 7 8 11 13 14], [19 20]);
end
B_longitude_lo = mat_lo([3 5 7 8 11 13 14], [19 20]);

%% Select the components that make up the longitude C matrix
%%
if ho_value_on
    C_longitude_hi = mat_hi([21 23 25 26 29], [3 5 7 8 11 13 14]);
end
C_longitude_lo = mat_lo([21 23 25 26 29], [3 5 7 8 11 13 14]);

%% Select the components that make up the longitude D matrix
%%
if ho_value_on
    D_longitude_hi = mat_hi([21 23 25 26 29], [19 20]);
end
D_longitude_lo = mat_lo([21 23 25 26 29], [19 20]);

if ho_value_on
    SS_long_hi = ss(A_longitude_hi, B_longitude_hi, C_longitude_hi, D_longitude_hi);
end
SS_long_lo = ss(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo);

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lateral Directional %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Select the components that make up the lateral A matrix
%%
if ho_value_on
    A_lateral_hi = mat_hi([4 6 7 9 10 12 13 15 16], [4 6 7 9 10 12 13 15 16]);
end
A_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [4 6 7 9 10 12 13 15 16]);

%% Select the components that make up the lateral B matrix
%%
if ho_value_on
    B_lateral_hi = mat_hi([4 6 7 9 10 12 13 15 16], [19 21 22]);
end 
B_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [19 21 22]);

%% Select the components that make up the lateral C matrix
%%
if ho_value_on
    C_lateral_hi = mat_hi([22 24 25 27 28 30], [4 6 7 9 10 12 13 15 16]);
end
C_lateral_lo = mat_lo([22 24 25 27 28 30], [4 6 7 9 10 12 13 15 16]);

%% Select the components that make up the lateral D matrix
%%
if ho_value_on
    D_lateral_hi = mat_hi([22 24 25 27 28 30], [19 21 22]);
end
D_lateral_lo = mat_lo([22 24 25 27 28 30], [19 21 22]);

if ho_value_on
    SS_lat_hi = ss(A_lateral_hi, B_lateral_hi, C_lateral_hi, D_lateral_hi);
end
SS_lat_lo = ss(A_lateral_lo, B_lateral_lo, C_lateral_lo, D_lateral_lo);

%% Make longitudal direction SYSTEM matrix
%%
if ho_value_on
    sys_long_hi = pck(A_longitude_hi, B_longitude_hi, C_longitude_hi, D_longitude_hi);
end 
sys_long_lo = pck(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo);



%% Make lateral direction SYSTEM matrix and Find poles for hifi
%%
if ho_value_on
    sys_lat_hi = pck(A_lateral_hi, B_lateral_hi, C_lateral_hi, D_lateral_hi);

    long_poles_hi = spoles(sys_long_hi);
    lat_poles_hi = spoles(sys_lat_hi);
end

%% Make lateral direction SYSTEM matrix and Find poles for lofi
%%
sys_lat_lo = pck(A_lateral_lo, B_lateral_lo, C_lateral_lo, D_lateral_lo);

long_poles_lo = spoles(sys_long_lo);
lat_poles_lo = spoles(sys_lat_lo);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Display results

if display_results
    clc;

    disp(sprintf('Altitude: %.3f ft.', altitude));
    disp(sprintf('Velocity: %.3f ft/s\n\n', velocity));
    if ho_value_on
        disp('For HIFI Model:  ');
        disp('Longitudal Direction:  ');
        disp(newline);

        disp('A =')
        for i=1:length( A_longitude_hi(:,1) )
            mprintf([ A_longitude_hi(i,:) ],'  %.3e ')
        end %for

        disp('B =')
        for i=1:length( B_longitude_hi(:,1) )
            mprintf([ B_longitude_hi(i,:) ],'  %.3e ')
        end %for

        disp('C =')
        for i=1:length( C_longitude_hi(:,1) )
            mprintf([ C_longitude_hi(i,:) ],'  %.3e ')
        end %for

        disp('D =')
        for i=1:length( D_longitude_hi(:,1) )
            mprintf([ D_longitude_hi(i,:) ],'  %.3e ')
        end %for

        rifd(long_poles_hi)

        disp(newline);

        disp('Lateral Direaction:  ');

        disp(newline);

        disp('A =')
        for i=1:length( A_lateral_hi(:,1) )
            mprintf([ A_lateral_hi(i,:) ],'  %.3e ')
        end %for

        disp('B =')
        for i=1:length( B_lateral_hi(:,1) )
            mprintf([ B_lateral_hi(i,:) ],'  %.3e ')
        end %for

        disp('C =')
        for i=1:length( C_lateral_hi(:,1) )
            mprintf([ C_lateral_hi(i,:) ],'  %.3e ')
        end %for

        disp('D =')
        for i=1:length( D_lateral_hi(:,1) )
            mprintf([ D_lateral_hi(i,:) ],'  %.3e ')
        end %for

        rifd(lat_poles_hi)

        disp(newline);
        disp(newline);
    end 
    disp('For LOFI Model:  ');
    disp('Longitudal Direction:  ');
    disp(newline);

    disp('A =')
    for i=1:length( A_longitude_lo(:,1) )
        mprintf([ A_longitude_lo(i,:) ],'  %.3e ')
    end %for

    disp('B =')
    for i=1:length( B_longitude_lo(:,1) )
        mprintf([ B_longitude_lo(i,:) ],'  %.3e ')
    end %for

    disp('C =')
    for i=1:length( C_longitude_lo(:,1) )
        mprintf([ C_longitude_lo(i,:) ],'  %.3e ')
    end %for

    disp('D =')
    for i=1:length( D_longitude_lo(:,1) )
        mprintf([ D_longitude_lo(i,:) ],'  %.3e ')
    end %for

    % Display the real, imaginary, frequency (magnitude) and damping ratios
    rifd(long_poles_lo)

    disp(newline);

    disp('Lateral Direaction:  ');

    disp(newline);

    disp('A =')
    for i=1:length( A_lateral_lo(:,1) )
        mprintf([ A_lateral_lo(i,:) ],'  %.3e ')
    end %for

    disp('B =')
    for i=1:length( B_lateral_lo(:,1) )
        mprintf([ B_lateral_lo(i,:) ],'  %.3e ')
    end %for

    disp('C =')
    for i=1:length( C_lateral_lo(:,1) )
        mprintf([ C_lateral_lo(i,:) ],'  %.3e ')
    end %for

    disp('D =')
    for i=1:length( D_lateral_lo(:,1) )
        mprintf([ D_lateral_lo(i,:) ],'  %.3e ')
    end %for

    % Display the real, imaginary, frequency (magnitude) and damping ratios
    rifd(lat_poles_lo)
else
%% Reduce state space for longitudinal %%%%%%%%%%%%%%%%%%%%
% State space of four states (chapter 5) 
ordered_A = A_longitude_lo([3,4,2,5,7,6],[3,4,2,5,7,6]);
long_A = ordered_A([1,2,3,4],[1,2,3,4]);
long_B = ordered_A([1,2,3,4],5);
long_C = C_longitude_lo([3,4,2,5],[3,4,2,5]);
long_D = D_longitude_lo([3,4,2,4],2);
ss_long = ss(long_A, long_B, long_C, long_D);

% State space of two states (chapter 6) 
two_stateA = long_A([2,4],[2,4]);
two_stateB = long_B([2,4]);
two_stateC = long_C([2,4],[2,4]);
two_stateD = long_D([2,4]);
ss_two_long = ss(two_stateA, two_stateB, two_stateC, two_stateD);

% To get the correct responses
tf_four_state = tf(ss_long(4,1));
tf_two_state = tf(ss_two_long(2,1));

[num,den] = tfdata(tf_two_state,'v');

% Get step input response
%step_input_response = step(tf_four_state, tf_two_state, 15);

% Initially calculated components
k_q_init= num(3);
T_theta_init = num(2)/k_q_init
omega_n_init = sqrt(den(3));
zeta_sp_init = den(2)/(2*omega_n_init);

% Requirement values
velocity_ms = velocity * 0.3048;
omega_n_req = 0.03*velocity_ms;
zeta_sp_req = 0.5;
T_theta_req = 1/(0.75*omega_n_req)
% kf*(0.243s+1)/ (2.2767s+1)
% Get the gains such that the requirements are fullfilled
denominator_standard = [1 2*omega_n_req*zeta_sp_req omega_n_req^2];
desired_poles = roots(denominator_standard);
k = place(two_stateA, two_stateB, desired_poles);
k_alpha = k(1);
k_q = k(2); 

% Convert vertical gust disturbance to angle of attack 
gust_disturbance = 4.572; % m/s
angle_of_attack = atan(gust_disturbance/(velocity_ms));  % radians

% Find the elevator deflection due to an angle of attack
elevator_deflection_ref = xu_lo(14);  % radians
pitch_rate_ref = 0;  % radians/s
elevator_deflection_tot = elevator_deflection_ref - angle_of_attack*k_alpha - pitch_rate_ref*k_q

% Desired transfer function
desired_tf = tf([0, T_theta_req*k_q, k_q], [1, 2*zeta_sp_req*omega_n_req, omega_n_req^2])

% Updated tranfer function with the new poles
updated_tf = tf([num(1), num(2), num(3)], [1, 2*zeta_sp_req*omega_n_req, omega_n_req^2])  % Fill in
gain = 1;  % change gain
filter = tf([T_theta_req, 1],[T_theta_init 1])*gain



%% Calculate CAP and DB/qs
%CAP
cap  = omega_n_req^2 / (velocity_ms/(9.80665*T_theta_req));
dbqs = T_theta_req-1/omega_n_req;


%create latex code
sympref('FloatingPointOutput',true)
Latex_long_A = latex(sym(round(long_A,3)));
Latex_long_B = latex(sym(round(long_B,3)));
Latex_long_C = latex(sym(round(long_C,3)));
Latex_long_D = latex(sym(round(long_D,3)));

%% Reduce state space for lateral %%%%%%%%%%%%%%%%%%%%
ordered_A_LAT = A_lateral_lo([4,1,5,6,8,9],[4,1,5,6,8,9]);
lat_A = ordered_A_LAT([1,2,3,4],[1,2,3,4]);
lat_B = ordered_A_LAT([1, 2, 3, 4],[5,6]);
lat_C = C_lateral_lo([4,1,5,6],[4,1,5,6]);
lat_D = D_lateral_lo([4,1,5,6],[2,3]);
ss_lat = ss(lat_A, lat_B, lat_C, lat_D);

%create latex code
Latex_long_A = latex(sym(round(long_A,3)));
Latex_long_B = latex(sym(round(long_B,3)));
Latex_long_C = latex(sym(round(long_C,3)));
Latex_long_D = latex(sym(round(long_D,3)));

%create latex code
Latex_A = latex(sym(round(A_lateral_lo,3)));
Latex_B = latex(sym(round(B_lateral_lo,3)));
Latex_C = latex(sym(round(C_lateral_lo,3)));
Latex_D = latex(sym(round(D_lateral_lo,3)));

if plotting == true
    figure(1);
    pzmap(ss_long, 'b');
    title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nPole-plot\n Longitudinal Eigenmotions', altitude, velocity);
    title(title_string);
    sgrid;
    figure(2);
    pzmap(ss_lat, 'b');
    title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nPole-plot\n Lateral Eigenmotions', altitude, velocity);
    title(title_string);
    sgrid;
end
[eig_vec_long,longitudinal_eig] = eig(long_A);
[eig_vec_lat,lateral_eig] = eig(lat_A);

% Eigenmotions Longitudinal
eigenmotion1 = longitudinal_eig(1,1);
eigenmotion2 = longitudinal_eig(3,3);

if (eigenmotion1 < eigenmotion2);
    short_period = eigenmotion1;
    phugoid = eigenmotion2;
    
else 
    short_period = eigenmotion2;
    phugoid = eigenmotion1;
end

%% Eigenmotions lateral

i=1;
j=1;

for k = 1 : 4;
    if (imag(lateral_eig(k,k)) == 0);
        roll_spiral(i) = lateral_eig(k,k);
        i=i+1;
    else
        dutch_roll = lateral_eig(k,k);
        j=j+1;
    end
end

if roll_spiral(1) < roll_spiral(2);
    
    roll = roll_spiral(1);
    spiral = roll_spiral(2);
    
else
    spiral = roll_spiral(1);
    roll = roll_spiral(2);
    
end

%% periodic Eigenmotions
%short period
omega_n_shortperiod = sqrt( real(short_period)^2 + imag(short_period)^2);
zeta_shortperiod = -real(short_period)/ omega_n_shortperiod;
period_shortperiod = 2*pi/abs(imag(short_period));
t_half_shortperiod = log(2)/ abs(real(short_period));

%phugoid
omega_n_phugoid = sqrt( real(phugoid)^2 + imag(phugoid)^2);
zeta_phugoid = -real(phugoid)/ omega_n_phugoid;
period_phugoid = 2*pi/abs(imag(phugoid));
t_half_phugoid = log(2)/ abs(real(phugoid));
%dutch roll
omega_n_dutch_roll = sqrt( real(dutch_roll)^2 + imag(dutch_roll)^2);
zeta_dutch_roll = -real(dutch_roll)/ omega_n_dutch_roll;
period_dutch_roll = 2*pi/abs(imag(dutch_roll));
t_half_dutch_roll = log(2)/ abs(real(dutch_roll));


%% aperiodic Eigenmotions
%aperiodic roll
timeconst_roll = -1/real(roll);
omega_roll = sqrt( real(roll)^2 + imag(roll)^2);
t_half_roll = log(2)/ abs(real(roll));
%spiral
timeconst_spiral = -1/real(spiral);
omega_spiral = sqrt( real(spiral)^2 + imag(spiral)^2);
t_half_spiral = log(2)/ abs(real(spiral));

% Create a table which can be used in latex for the periodic motions
periodic_eigenmotions_values = [omega_n_shortperiod, zeta_shortperiod, period_shortperiod, t_half_shortperiod;
    omega_n_phugoid, zeta_phugoid, period_phugoid, t_half_phugoid;
    omega_n_dutch_roll, zeta_dutch_roll, period_dutch_roll, t_half_dutch_roll];

% Create a table which can be used in latex for the aperiodic motions
aperiodic_eigenmotions_values = [omega_roll, timeconst_roll, t_half_roll;
    omega_spiral, timeconst_spiral, t_half_spiral];

%% plotting eigenmotion response 

x_sp = eig_vec_long(:,1)+eig_vec_long(:,2);
x_ph = eig_vec_long(:,3)+eig_vec_long(:,4);
t_shortperiod = 0: 0.001:7;
t_phugoid = 0:0.1:500;

y_shortperiod   = zeros(4, length(t_shortperiod));
y_phugoid       = zeros(4, length(t_phugoid));
for i=1:length(t_shortperiod)
    y_shortperiod(:,i) = long_C*expm(t_shortperiod(i)*long_A)*x_sp;  
end

for i=1:length(t_phugoid)
   y_phugoid(:,i) = long_C*expm(t_phugoid(i)*long_A)*x_ph; 
end



% Plotting Longitudinal Responses
ylabels    = ["Velocity [ft/s]", "\alpha [deg]", "\theta [deg]", "q [deg/s]"];
titles_sp   = ["\Delta V", "\Delta \alpha", "\Delta \theta", "\Delta q"];
titles_ph   = ["\Delta V", "\Delta \alpha", "\Delta \theta", "\Delta q"];
nr=4;
list_longitudinal = ["Time Response - Short Period", "Time Response - Phugoid"];
if eigenmotion_plots == true
    for i = 1:2 
        figure(nr)
        nr = nr+1;
        for j = 1:4
            if i == 1
                subplot(4,1,j)
                plot(t_shortperiod, y_shortperiod(j,:))
                xlabel('Time [sec]')
                ylabel(ylabels(j))
                title(titles_sp(j))
                sgtitle(list_longitudinal(i))
                if saving == true
                    saveas(gcf,list_longitudinal(i)+'.png')
                end
            else
                subplot(4,1,j)
                plot(t_phugoid, y_phugoid(j,:))
                xlabel("Time [sec]")
                ylabel(ylabels(j))
                title(titles_ph(j))
                sgtitle(list_longitudinal(i))
                if saving == true
                    saveas(gcf,list_longitudinal(i)+'.png')
                end
            end
        end
    end
end



x_ap = eig_vec_lat(:,3);
x_dr = eig_vec_lat(:,1)+eig_vec_lat(:,2);
x_sm = eig_vec_lat(:,4);

list_motions_lat = ["Time Response - Aperiodic Roll", "Time Response - Dutch Roll","Time Response - Spiral"];


% Create time vector(s)
time_aperiodic = 0:0.01:4;
time_dutchroll = 0:0.01:16;
time_spiral = 0:0.01:600;

y_aperiodic = zeros(4, length(time_aperiodic));
y_dutchroll = zeros(4, length(time_dutchroll));
y_spiral = zeros(4, length(time_spiral));

% Calculating eigenmotion responses based on eigenvectors xap, xdr and xsm
for i=1:length(time_aperiodic)
    y_aperiodic(:,i) = lat_C*expm(time_aperiodic(i)*lat_A)*x_ap;
end

for i=1:length(time_dutchroll)
    y_dutchroll(:,i) = lat_C*expm(time_dutchroll(i)*lat_A)*x_dr;
end

for i=1:length(time_spiral)
    y_spiral(:,i) = lat_C*expm(time_spiral(i)*lat_A)*x_sm;
end

% Plotting Lateral responses
ylabels    = ["\beta [deg]", "\phi [deg]", "p [deg/s]", "r [deg/s]"];
titles_ap   = ["\Delta \beta", "\Delta \phi", "\Delta p", "\Delta r"];
titles_dr   = ["\Delta \beta", "\Delta \phi", "\Delta p", "\Delta r"];
titles_sm   = ["\Delta \beta", "\Delta \phi", "\Delta p", "\Delta r"];

if eigenmotion_plots == true
    for i = 1:3 
        figure(nr)
        nr = nr+1;
        for j = 1:4
            if i == 1
                subplot(4,1,j)
                plot(time_aperiodic, y_aperiodic(j,:));
                xlabel('Time [sec]')
                ylabel(ylabels(j))
                %title(titles_ap(j))
                sgtitle(list_motions_lat(i))
                grid on;
                if saving == true
                    saveas(gcf,list_motions_lat(i)+'.png')
                end
            elseif i == 2
                subplot(4,1,j)
                plot(time_dutchroll, y_dutchroll(j,:));
                xlabel("Time [sec]")
                ylabel(ylabels(j))
                %title(titles_dr(j))
                sgtitle(list_motions_lat(i))
                grid on;
                if saving == true
                    saveas(gcf,list_motions_lat(i)+'.png')
                end
            else 
                subplot(4,1,j)
                plot(time_spiral, y_spiral(j,:));
                xlabel("Time [sec]")
                ylabel(ylabels(j))
                sgtitle(list_motions_lat(i))
                %title(titles_sm(j))
                grid on;
                if saving == true
                    saveas(gcf,list_motions_lat(i)+'.png')
                end
            end
                %sgtitle(list_motions_lat(i));
                %saveas(gcf,list_motions_lat(i)+'.png');
        end
    end
end

% Creation of Matrix B by using a part of matrix A. Also, reduction of
% matrix A by taking out the elevator deflection and ignoring thrust


%     long_A = latex_creator(A_longitude_lo, 1)
%     long_B = latex_creator(B_longitude_lo, 2)
%     long_C = latex_creator(C_longitude_lo, 3)
%     long_D = latex_creator(D_longitude_lo, 4)
%     

% %     
%     lat_A = latex_creator(A_lateral_lo, 5)
%     lat_B = latex_creator(B_lateral_lo, 6)
%     lat_C = latex_creator(C_lateral_lo, 7)
%     lat_D = latex_creator(D_lateral_lo, 8)
%     
%     Latex_lat_A = latex(sym(lat_A))
%     Latex_lat_B = latex(sym(lat_B))
%     Latex_lat_C = latex(sym(lat_C))
%     Latex_lat_D = latex(sym(lat_D))

    %transfer_function_acc = tf(SS_lo(19,2))
    %z = zero(transfer_function_acc)
    disp('FINISHED')
end    

if plotting
    %% All Poles
    figure(1); 
    if ho_value_on
        pzmap(SS_hi, 'r', SS_lo, 'b');
        title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nAll Poles\n Blue = lofi Red = hifi.', altitude, velocity);
    else
        pzmap(SS_lo, 'b');
        title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nAll Poles\n Blue = lofi R', altitude, velocity);
    end
    title(title_string);
    sgrid;

    %% Long. Poles
    %%
    figure(2); 
    if ho_value_on
        pzmap(SS_long_hi, 'r', SS_long_lo, 'b');
        title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLongitudal Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
    else
        pzmap(SS_long_lo, 'b');
        title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLongitudal Directional Poles\n Blue = lofi', altitude, velocity);
    end
    title(title_string);
    sgrid;

    %% Lat. Poles
    %%
    figure(3); 
    if ho_value_on
        pzmap(SS_lat_hi, 'r', SS_lat_lo, 'b');
        title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLateral Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
    else
        pzmap(SS_lat_lo, 'b');
        title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLateral Directional Poles\n Blue = lofi', altitude, velocity);
    end
    title(title_string);
    sgrid;

    % Create Bode Plots

    omega = logspace(-2,2,100);
    if ho_value_on
        sysg_lat_hi = frsp(sys_lat_hi,omega);
    end
    sysg_lat_lo = frsp(sys_lat_lo,omega);

    if ho_value_on
        sysg_long_hi = frsp(sys_long_hi,omega);
    end
    sysg_long_lo = frsp(sys_long_lo,omega);

    figure;
    BodeCount = 0;
    for state = 1:1:5
        for control = 1:1:2
            BodeCount = BodeCount +1;
            title_string = sprintf('Bode Plot #%d\n State = %d\n Control = %d', BodeCount,state,control);
            disp(title_string);
            if ho_value_on
                vplot('bode', sel(sysg_long_hi,state,control), 'b--', sel(sysg_long_lo,state,control), 'r');
                legend('hifi', 'lofi');
            else
                vplot('bode', sel(sysg_long_lo,state,control), 'r');
                legend('lofi');
            end 
            %pause;
        end
    end

    for state = 1:1:6
        for control = 1:1:3
            BodeCount = BodeCount + 1;
            title_string = sprintf('Bode Plot #%d\n State = %d\n Control = %d', BodeCount,state,control);
            disp(title_string);
            if ho_value_on
                vplot('bode', sel(sysg_lat_hi,state,control), 'b--', sel(sysg_lat_lo,state,control), 'r');
                legend('hifi', 'lofi');
            else
                vplot('bode', sel(sysg_lat_lo,state,control), 'r');
                legend('lofi');
            end 
            %pause;
        end
    end
end

%% Chapter 7
%
ordered_A = A_longitude_lo([1,3,4,2,5,6,7],[1,3,4,2,5,6,7]);
long_A = ordered_A([1,2,3,4,5],[1,2,3,4,5]);
long_B = ordered_A([1,2,3,4,5],[6,7]);
long_C = C_longitude_lo([1,3,4,2,5],[1,3,4,2,5]);
long_D = D_longitude_lo([1,3,4,2,5],[1,2]);
ss_long = ss(long_A, long_B, long_C, long_D)

