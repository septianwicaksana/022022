function dxdt = vehicleCT0(x,u)
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
% 
% const1 = (2*reff*u(1)-2*x(4))/(reff*u(1));
% const2 = ((x(5)-lf*x(6))/(x(4)+1e-6));
% const3 = ((x(5)-lr*x(6))/(x(4)+1e-6));
% 
% dxdt = x;
% dxdt(1) = x(4) + 0; % Vx
% dxdt(2) = x(5) + 0; % Vy
% dxdt(3) = x(6) + 0; % Vz
% dxdt(4) = (-x(5)*x(6)-(sin(x(4))/m)*(1/2*ro*Cdrag*Afront*x(4)*x(4)+fr*m*g+m*g*sin(Bx))) + (1/m*(Cdf*const1*cos(u(2))-2*Caf*(u(2)-const2)*sin(u(2))+ Cdf*const1));
% dxdt(5) = (-x(4)*x(6)+1/m*(2*Car*(-const3)-g*sin(By)))+ (1/m*(Cdf*const1*sin(u(2))-2*Caf*(u(2)-const2)*cos(u(2))));
% dxdt(6) = (lr/Iz*(2*Car/m*(-const3)))+ (lf/Iz*(Cdf*const1*sin(u(2))-2*Caf*(u(2)-const2)*cos(u(2))));
    u(1) = u(1)+1e-6;
    const1 = (2*reff*u(1)-2*x(4))/(reff*u(1));
    const2 = ((x(5)-lf*x(6))/(x(4)+1e-6));
    const3 = ((x(5)-lr*x(6))/(x(4)+1e-6));
    
    f41 = -x(5)*x(6);
    f42 = -(sin(x(4))/m)*(1/2*ro*Cdrag*Afront*x(4)*x(4));
    f43 = -(sin(x(4))/m)*(fr*m*g);
    f44 = -(sin(x(4))/m)*(m*g*sin(Bx));
    u41 = 1/m*(Cdf*const1*cos(u(2)));
    u42 = 1/m*(-2*Caf*(u(2)-const2)*sin(u(2)));
    u43 = 1/m*(Cdf*const1);
    
    f51=-x(4)*x(6);
    f52=1/m*(2*Car*(-const3));
    f53=1/m*(-g*sin(By));
    u51=1/m*(Cdf*const1*sin(u(2)));
    u52=1/m*(-2*Caf*(u(2)-const2)*cos(u(2)));
    
    f61=lr/Iz*(2*(Car/m)*(-const3));
    u61=lf/Iz*(Cdf*const1*sin(u(2)));
    u62=lf/Iz*(-2*Caf*(u(2)-const2)*cos(u(2)));
    
    dxdt(1) = x(4); % Vx
    dxdt(2) = x(5); % Vy
    dxdt(3) = x(6); % Vz
    dxdt(4) = f41+f42+f43+u41+u42+u43;
    dxdt(5) = f51+f52+f53+u51+u52;
    dxdt(6) = f61+u61+u62;
