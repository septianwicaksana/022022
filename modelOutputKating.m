clear
clc

Wwheel = load('Ww.mat').ans';
SteerAngle = load('steer.mat').ans';

%parameter
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
rst = 0.302 ;
rnom = 0.302*0.98 ;
Cdf = 55;
Caf = 0.09;
Car = 0.09;
B = acosd(rst/rnom);
reff = sin(B)/B * rnom;
% TLim=2;
Ts = 0.01;
t = 0:Ts:10-0.001;
% Ts=0.005;
x=zeros(6,1);
dx=zeros(6,1);
% u=[1 0];

for i=1:2658
    % State Equations
    %     u1=Wwheel(i);
%     u1=Wwheel;
    u1=Wwheel(i)+1e-6;
    u2=SteerAngle(i);
    u=[u1 u2];
%     
    const1 = (2*reff*u(1)-2*x(4,i))/(reff*u(1));
%   const1 = (2*reff*u(1)-2*x(4,i));
    const2 = ((x(5,i)-lf*x(6,i))/(x(4,i)+1e-6));
    const3 = ((x(5,i)-lr*x(6,i))/(x(4,i)+1e-6));
    
    f41 = -x(5,i)*x(6,i);
    f42 = -(sin(x(4,i))/m)*(1/2*ro*Cdrag*Afront*x(4,i)*x(4,i));
    f43 = -(sin(x(4,i))/m)*(fr*m*g);
    f44 = -(sin(x(4,i))/m)*(m*g*sin(Bx));
    u41 = 1/m*(Cdf*const1*cos(u(2)));
    u42 = 1/m*(-2*Caf*(u(2)-const2)*sin(u(2)));
    u43 = 1/m*(Cdf*const1);
    
    f51=-x(4,i)*x(6,i);
    f52=1/m*(2*Car*(-const3));
    f53=1/m*(-g*sin(By));
    u51=1/m*(Cdf*const1*sin(u(2)));
    u52=1/m*(-2*Caf*(u(2)-const2)*cos(u(2)));
    
    f61=lr/Iz*(2*(Car/m)*(-const3));
    u61=lf/Iz*(Cdf*const1*sin(u(2)));
    u62=lf/Iz*(-2*Caf*(u(2)-const2)*cos(u(2)));
    
    dx(1,i) = x(4,i); % Vx
    dx(2,i) = x(5,i); % Vy
    dx(3,i) = x(6,i); % Vz
    dx(4,i) = f41+f42+f43+u41+u42+u43;
    dx(5,i) = f51+f52+f53+u51+u52;
    dx(6,i) = f61+u61+u62;
    
    x(1,i+1)=x(1,i)+Ts.*dx(1,i);
    x(2,i+1)=x(2,i)+Ts.*dx(2,i);
    x(3,i+1)=x(3,i)+Ts.*dx(3,i);
    x(4,i+1)=x(4,i)+Ts.*dx(4,i);
    x(5,i+1)=x(5,i)+Ts.*dx(5,i);
    x(6,i+1)=x(6,i)+Ts.*dx(6,i);
    
end
% label = [{'Vx'},{'Vy'},{'Vz'},{'ax'},{'ay'},{'az'}]
% plot posisi XY
% for i = 1:6
%     figure(i)
%     plot(x(i,:));
%     title(label{i})
% end
figure(1)
plot(x(1,:),x(2,:))
figure(2)
plot(x(4,:))
figure(3)
plot(x(5,:))
figure(4)
plot(x(6,:))
% figure(1)
% plot(x(1,:)