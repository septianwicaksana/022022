%%

% clc;clear;close all;
Xdot = load('Xdot.mat').ans'; 
Xdot = Xdot(1:2658);
Xdotdot = diff(Xdot);
Ydot = load('Ydot.mat').ans';
Ydot = Ydot(1:2658);
Ydotdot = diff(Ydot);
Zdot = load('Zdot.mat').ans';
Zdot = Zdot(1:2658);
Zdotdot = diff(Zdot);

nx = 6;
ny = 3;
nu = 2;
nlobj = nlmpc(nx,ny,nu);

Ts = 1/264.8;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 20;
nlobj.ControlHorizon =10;

%%
% function -> "vehicleDT0"

nlobj.Model.StateFcn = "vehicleDT0";
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;

% function -> "vehicleOutputFcn"

nlobj.Model.OutputFcn = "vehicleOutputFcn";
nlobj.Jacobian.OutputFcn = @(x,u,Ts) [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];
nlobj.Weights.OutputVariables = [3 3 3];
nlobj.Weights.ManipulatedVariablesRate = [0.1 0.1];


% nlobj.MV(1).Min = -3;      % Maximum acceleration 3 m/s^2
% nlobj.MV(1).Max = 3;       % Minimum acceleration -3 m/s^2
% nlobj.MV(2).Min = -1.13;   % Minimum steering angle -65
% nlobj.MV(2).Max = 1.13;    % Maximum steering angle 65


% nlobj.OV(1).ScaleFacStor = 15;   % Typical value of longitudinal velocity
% nlobj.OV(2).ScaleFactor = 0.5;  % Range for lateral deviation
% nlobj.OV(3).ScaleFactor = 0.5;  % Range for relative yaw angle
% nlobj.MV(1).ScaleFactor = 6;    % Range of steering angle
% nlobj.MV(2).ScaleFactor = 2.26; % Range of acceleration

x0 = [rand;rand;rand;rand;rand;rand];
u0 = [0.4;0.2];
validateFcns(nlobj,x0,u0,[],{Ts});

%%
% function -> "vehicleStateFcn"
% function -> "vehicleMeasuremntFcn"

EKF = extendedKalmanFilter(@vehicleStateFcn, @vehicleMeasurementFcn);

x = [0.1;0.1;0.1;0.1;0.1;0.1];
y = [x(1);x(2);x(3)];
EKF.State = x;

mv = [0.1 0.1];
yref = [0 0 0];

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

Duration = 10;
hbar = waitbar(0,'Simulation Progress');
xHistory = x;

for ct = 1:(10/Ts)
    yref = [Xdot(ct) Ydot(ct) Zdot(ct)];
    % Correct previous prediction using current measurement.
    xk = correct(EKF, y);
    % Compute optimal control moves.
    [mv,nloptions,info] = nlmpcmove(nlobj,xk,mv,yref,[],nloptions);
   
    % Predict prediction model states for the next iteration.
    predict(EKF, [mv; Ts]);
    % Implement first optimal control move and update plant states.
    x = vehicleDT0(x,mv,Ts);
    % Generate sensor data with some white noise.
    y = x([1 3 1]) + randn(3,1)*0.01; 
    % Save plant states for display.
    xHistory = [xHistory x]; %#ok<*AGROW>
    waitbar(ct*Ts/10,hbar);
end
close(hbar)

%%

figure(1)
subplot(3,1,1);
plot(0:Ts:Duration,xHistory(1,:))
xlabel('time')
ylabel('m/s')
title('Vx')
hold on
plot(0:Ts:Duration,Xdot(1:2649))
xlabel('time')
ylabel('m/s^2')
title('Vx')


subplot(3,1,2); 
plot(0:Ts:Duration,xHistory(2,:))
xlabel('time')
ylabel('m/s')
title('Vy')
hold on 
plot(0:Ts:Duration,Ydot(1:2649))
xlabel('time')
ylabel('m/s^2')
title('Vx')

subplot(3,1,3);
plot(0:Ts:Duration,xHistory(3,:))
xlabel('time')
ylabel('m/s')
title('Vy')
hold on 
plot(0:Ts:Duration,Zdot(1:2649))
xlabel('time')
ylabel('m/s^2')
title('Vx')

figure(2)
subplot(3,1,1);
plot(0:Ts:Duration,xHistory(4,:))
xlabel('time')
ylabel('m/s')
title('ax')
subplot(3,1,2);
plot(0:Ts:Duration,xHistory(5,:))
xlabel('time')
ylabel('m/s')
title('ay')
subplot(3,1,3);
plot(0:Ts:Duration,xHistory(6,:))
xlabel('time')
ylabel('m/s')
title('az')


