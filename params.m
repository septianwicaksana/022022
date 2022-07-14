clc;clear;
t = 0:1/265.9:10;
t = t(2:2660);
u1 = load('Ww.mat').ans';
u2 = load('steer.mat').ans';
u1 = [t',u1'];
u2 = [t',u2'];
Xdot = load('Xdot.mat').ans'; 
Ydot = load('Ydot.mat').ans';
Zdot = load('Zdot.mat').ans';
Xdot = [t',Xdot'];
Ydot = [t',Ydot'];
Zdot = [t',Zdot'];
m = 1500 + 2*80 + 300;
rhoAir = 1.225;
cDrag = 0.3;
aFront = 2.2;
fr = 0.02;
g = 9.81;
Bx = 0;
By = 0;
lf = 1.22;
lr = 1.36;
Iz = 1536.70;
rst = 0.302 ; % jari jari roda statis, nilai didapatkan dari jurnal 4 steering
rnom = 0.302*0.98 ; % 2 persen dari jari - jari roda statis
cZigmaFront = 55;
cAlphaFront = 0.09;
cAlphaR = 0.09;
B = acosd(rst/rnom);
reff = sin(B)/B * rnom;
nlobj = nlmpc(6,6,'MV',[1 2]);

%%
% Specify the controller sample time, prediction horizon, and control
% horizon. 
nlobj.Ts = 0.1;
% nlobj.PredictionHorizon = 10;
% nlobj.ControlHorizon = 2;

%% 
% Specify the state function for the nonlinear plant model and its
% Jacobian.
nlobj.Model.StateFcn = @(x,u) LaneFollowingVehicleModel(x,u);

%% 
% Specify the output function for the nonlinear plant model and its
% Jacobian. The output variables are:
%
% * Longitudinal velocity
% * Lateral deviation
% * Sum of the yaw angle and yaw angle output disturbance
%
nlobj.Model.OutputFcn = @(x,u) [x(1);x(2);x(3);x(4);x(5);x(6)];

%% 
% Set the constraints for manipulated variables.
nlobj.MV(1).Min = -3;      % Maximum acceleration 3 m/s^2
nlobj.MV(1).Max = 3;       % Minimum acceleration -3 m/s^2
nlobj.MV(2).Min = -1.13;   % Minimum steering angle -65 
nlobj.MV(2).Max = 1.13;    % Maximum steering angle 65


%%
x0 = [0.1 0.5 25 0.1 0.1 0.001];
u0 = [0.125 0.4];
ref0 = [0 0 0 0 0 0];
validateFcns(nlobj,x0,u0,[],{},ref0);
