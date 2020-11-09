
TF0 = tf([0    0.4210   -1.8400  -22.1497    0.0770    0.0012   -0.0000],[1.0000   21.7321   33.0006   41.4799    0.5546    0.2930   -0.0000]);
TF5 = tf([0    0.0642   -2.0663  -22.1522    0.0770    0.0012   -0.0000],[1.0000   21.7321   33.0006   41.4799    0.5546    0.2930   -0.0000]);
TF59 = tf([0   -3.1377e-05   -2.1070  -22.1527    0.0770    0.0012    0.0000],[1.0000   21.7321   33.0006   41.4799    0.5546    0.2930   -0.0000]);
TF6 = tf([0   -0.0072   -2.1116  -22.1527    0.0770    0.0012   -0.0000],[1.0000   21.7321   33.0006   41.4799    0.5546    0.2930   -0.0000]);
TF7 = tf([0   -0.0785   -2.1568  -22.1532    0.0770    0.0012   -0.0000],[1.0000   21.7321   33.0006   41.4799    0.5546    0.2930   -0.0000]);
TF15 = tf([0   -0.6494   -2.5189  -22.1573    0.0770    0.0012   -0.0000],[1.0000   21.7321   33.0006   41.4799    0.5546    0.2930   -0.0000]);

% TF0 = minreal(TF0)
% TF5 = minreal(TF5)
% TF59 = minreal(TF59)
% TF6 = minreal(TF6)
% TF7 = minreal(TF7)
% TF15 = minreal(TF15)

Z0 = zero(TF0)
Z5 = zero(TF5)
Z59 = zero(TF59)
Z6 = zero(TF6)
Z7 = zero(TF7)
Z15 = zero(TF15)

table = [Z0(2) Z0(3) Z0(4) Z0(5) Z0(1) ;
Z5(2) Z5(3) Z5(4) Z5(5) Z5(1) ;
Z59(2) Z59(3) Z59(4) Z59(5) Z59(1);
Z6(2) Z6(3) Z6(4) Z6(5) Z6(1) ;
Z7(2) Z7(3) Z7(4) Z7(5) Z7(1) ;
Z15(2) Z15(3) Z15(4) Z15(5) Z15(1)]

xlswrite('table.xlsx',table)

t = 0:0.01:1;
y0 = step(t,-TF0);
y5 = step(t,-TF5);
y59 = step(t,-TF59);
y6 = step(t,-TF6);
y7 = step(t,-TF7);
y15 = step(t,-TF15);

figure(1)
plot(t,y0)
xlabel('Time');
ylabel('Normal Acceleration');
title('Time Response for a_n with x_a = 0ft');
grid on;
ylim([-0.05, 0.35]);
saveas(gcf,'x0.png');

figure(2)
plot(t,y5)
xlabel('Time');
ylabel('Normal Acceleration');
title('Time Response for a_n with x_a = 5ft');
grid on;
ylim([-0.05, 0.35]);
saveas(gcf,'x5.png');

figure(3)
plot(t,y59)
xlabel('Time');
ylabel('Normal Acceleration');
title('Time Response for a_n with x_a = 5.9ft');
grid on;
ylim([-0.05, 0.35]);
saveas(gcf,'x59.png');

figure(4)
plot(t,y6)
xlabel('Time');
ylabel('Normal Acceleration');
title('Time Response for a_n with x_a = 6ft');
grid on;
ylim([-0.05, 0.35]);
saveas(gcf,'x6.png');

figure(5)
plot(t,y7)
xlabel('Time');
ylabel('Normal Acceleration');
title('Time Response for a_n with x_a = 7ft');
grid on;
ylim([-0.05, 0.35]);
saveas(gcf,'x7.png');

figure(6)
plot(t,y15)
xlabel('Time');
ylabel('Normal Acceleration');
title('Time Response for a_n with x_a = 15ft');
grid on;
ylim([-0.05, 0.35]);
saveas(gcf,'x15.png');

figure(7)
plot(t, [y0,y5,y59,y6,y7,y15])
xlabel("Time [sec]")
ylabel("Normal acceleration [g units]")
legend("0ft","5ft","5.9ft","6ft","7ft","15ft")
title("Response")
grid on;
ylim([-0.05, 0.35]);
saveas(gcf,'xcombined.png');
