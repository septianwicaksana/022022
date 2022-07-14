function dxdt = LaneFollowingVehicleModel(x,u)
%% Vehicle Parameters
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

%% State Equations
dxdt = x;
dxdt(1) = x(4) + 0; % Vx
dxdt(2) = x(5) + 0; % Vy
dxdt(3) = x(6) + 0; % Vz
dxdt(4) = (-x(5)*x(6)-(sign(x(4))/m) * (1/2*ro*Cdrag*Afront*(x(4)^2) + fr*m*g + m*g*sin(Bx))) + (1/m*(Cdf*((2*reff*u(1)-2*x(4))/(reff*u(1))))*cos(u(2)) - (2*Caf*(u(2)-((x(5)-lf*x(6))/x(4)))*sin(u(2))) + Cdf*((2*reff*u(1)-2*x(4))/reff*u(1)));     % ax
dxdt(5) = (-x(4)*x(6) + 1/m*(2*Car*(-(x(5)-lr*x(6))/x(4))) - g*sin(By)) + (1/m*((Cdf*((2*reff*u(1)-2*x(4))/(reff*u(1)))*cos(u(2))) - 2*Caf*(u(2)-((x(5)-lf*x(6))/x(4)))*cos(u(2))));            % ay
dxdt(6) = (lr/Iz*(2*Car/m*(-(x(5)-lr*x(6))/x(4)))) + (lf/Iz*((Cdf*((2*reff*u(1)-2*x(4))/(reff*u(1)))*cos(u(2)))-2*Caf*(u(2)-((x(5)-lf*x(6))/x(4)))*cos(u(2))));                  % az

