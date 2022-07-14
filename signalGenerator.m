clear;clc;
x = [1 1 1 3 2 1.5];
% wwheel = load('Ww.mat').ans(10:2659);
% steerAngle = load('steer.mat').ans(10:2659);
wwheel = 20*ones(1,2650);
steerAngle = zeros(1,2650);

u = [wwheel;steerAngle]';

for i = 1: 2650
 xdot = vehicleCT0(x,u(i,:));
 x(i,1) = xdot(1);
 x(i,2) = xdot(2);
 x(i,3) = xdot(3);
 x(i,4) = xdot(4);
 x(i,5) = xdot(5);
 x(i,6) = xdot(6);
end

figure(1)
plot(x(:,4))
% hold on
% plot(Xdot)

figure(2)
plot(x(:,5))
% hold on
% plot(Ydot)

figure(3)
plot(x(:,6))
% hold on
% plot(Zdot)
