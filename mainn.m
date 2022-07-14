
%%
clc; clear all; close all;

Xdot = load('Xdot.mat').ans'; 
% Xdot = load('newLongitudinalVelo.mat').data.Data';
Ydot = load('Ydot.mat').ans';
% Ydot  = load('newLateralVelo.mat').data.Data';
Zdot = load('Zdot.mat').ans';
% Zdot = load('newYawVelo').data.Data';
Wwheel = load('Ww.mat').ans';
% Wwheel = Xdot/40;
SteerAngle = load('steer.mat').ans';
% SteerAngle = load('Delta.mat').data.Data';
% B = Vy./Vx;
% ar = lr .*Zdot./Vx - Vy./Vx
% t = load('time.mat').ans';
% xref = load('xref.mat').data.Data';
% yref = load('yref.mat').data.Data';
% xref = load('xPosition.mat').data.Data';
% yref = load('yPosition.mat').data.Data';

%%

[row,col] = size(Xdot);
x_dot = zeros(6,col);
m = 1500 + 2*80 + 300;
ro = 1.225;
Cdrag = 0.3;
Afront = 2.2;
fr = 0.02;
g = 9.81;
Bx = 0;
By = 0;
lf = 1.22;
lr = 1.36;
Iz = 1536.70;
rst = 0.302 ; % jari jari roda statis, nilai didapatkan dari jurnal 4 steering
rnom = 0.302*0.98 ; % 2 persen dari jari - jari roda statis
Cdf = 55;
Caf = 0.09;
Car = 0.09;
B = acosd(rst/rnom);
reff = sin(B)/B * rnom;

%%
X4 = 3
X5 = 2
X6 = 1.5
for i = 1 : col
%         X4 = Xdot(i);
%         X5 = Ydot(i);
%         X6 = Zdot(i);
        u1 = Wwheel(i);
        u2 = SteerAngle(i);

        x_dot(1,i) = X4 + 0;
        x_dot(2,i) = X5 + 0;
        x_dot(3,i) = X6 + 0;
        x_dot(4,i) = (-X5*X6-(sign(X4)/m) * (1/2*ro*Cdrag*Afront*(X4^2) + fr*m*g + m*g*sin(Bx))) + (1/m*(Cdf*((2*reff*u1-2*X4)/(reff*u1)))*cos(u2) - (2*Caf*(u2-((X5-lf*X6)/X4))*sin(u2)) + Cdf*((2*reff*u1-2*X4)/reff*u1));
        x_dot(5,i) = (-X4*X6 + 1/m*(2*Car*(-(X5-lr*X6)/X4)) - g*sin(By)) + (1/m*((Cdf*((2*reff*u1-2*X4)/(reff*u1))*cos(u2)) - 2*Caf*(u2-((X5-lf*X6)/X4))*cos(u2)));
        x_dot(6,i) = (lr/Iz*(2*Car/m*(-(X5-lr*X6)/X4))) + (lf/Iz*((Cdf*((2*reff*u1-2*X4)/(reff*u1))*cos(u2))-2*Caf*(u2-((X5-lf*X6)/X4))*cos(u2)));
        
        X4 = x_dot(4,i);
        X5 = x_dot(5,i);
        X6 = x_dot(6,i);

end

% A =[x4;x5;x6;-X5*X6-(sign(X4)/m)*(1/2*ro*Cdrag*Afront*(X4^2)+fr*m*g+m*g*sin(Bx)));-X4*X6+1/m*(2*Car*(-(X5-lr*X6)/X4))-g*sin(By));lr/Iz*(2*Car/m*(-(X5-lr*X6)/X4)))]
% B =[0;0;0;(1/m*(Cdf*((2*reff*u1-2*X4)/(reff*u1)))*cos(u2)-(2*Caf*(u2-((X5-lf*X6)/X4))*sin(u2))+Cdf*((2*reff*u1-2*X4)/reff*u1));(1/m*((Cdf*((2*reff*u1-2*X4)/(reff*u1))*cos(u2))-2*Caf*(u2-((X5-lf*X6)/X4))*cos(u2)));(lf/Iz*((Cdf*((2*reff*u1-2*X4)/(reff*u1))*cos(u2))-2*Caf*(u2-((X5-lf*X6)/X4))*cos(u2)))]
% C =[[0;0;0;1;0;0] [0;0;0;0;1;0] [0;0;0;0;0;1]]
% D = 

% label = [{'Vx'},{'Vy'},{'Vz'},{'ax'},{'ay'},{'az'}]
% for i = 1:6
%     figure(i)
%     plot(x_dot(i,:));
%     plot(x_dot(i,:));
%     title(label{i})
% end

figure(1)
plot(x_dot(1,:))
% hold on
% plot(Xdot)

figure(2)
plot(x_dot(2,:))
% hold on
% plot(Ydot)

figure(3)
plot(x_dot(3,:))
% hold on
% plot(Zdot)

% figure(7)
% plot(yref,xref);
% title('track');
% xlim([-80 80]);
% ylim([0 270]);

% figure(7)
% plot(xref,yref);
% title('track');
