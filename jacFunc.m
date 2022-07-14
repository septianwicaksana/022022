

clc;clear;
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

syms x1 x2 x3 x4 x5 x6 u1 u2

F = [x4+0;...
    x5+0;...
    x6+0;...
    (-x5*x6-(sign(x4)/m) * (1/2*ro*Cdrag*Afront*(x4^2) + fr*m*g + m*g*sin(Bx))) + (1/m*(Cdf*((2*reff*u1-2*x4)/(reff*u1)))*cos(u2) - (2*Caf*(u2-((x5-lf*x6)/x4))*sin(u2)) + Cdf*((2*reff*u1-2*x4)/reff*u1));...
    (-x4*x6 + 1/m*(2*Car*(-(x5-lr*x6)/x4)) - g*sin(By)) + (1/m*((Cdf*((2*reff*u1-2*x4)/(reff*u1))*cos(u2)) - 2*Caf*(u2-((x5-lf*x6)/x4))*cos(u2)));...
    (lr/Iz*(2*Car/m*(-(x5-lr*x6)/x4))) + (lf/Iz*((Cdf*((2*reff*u1-2*x4)/(reff*u1))*cos(u2))-2*Caf*(u2-((x5-lf*x6)/x4))*cos(u2)))]

J = jacobian(F,[x1,x2,x3,x4,x5,x6,u1,u2])
