clear
clc



%% load data

x=zeros(6,1);
dx=zeros(6,1);

% Wwheel = load('Ww.mat').ans';
% SteerAngle = load('steer.mat').ans';

filename = 'uScenarioOne.xlsx';
u = xlsread(filename);

Ts = 0.01;

%% calcualtion

for i=1:999
  
%     u1=Wwheel(i)+1e-6;
%     u2=SteerAngle(i);
%     u=[u1 u2];

    dx(:,i) = vehicleCT0Kating(x(:,i),u(i,:));
    
    x(1,i+1)=x(1,i)+Ts.*dx(1,i);
    x(2,i+1)=x(2,i)+Ts.*dx(2,i);
    x(3,i+1)=x(3,i)+Ts.*dx(3,i);
    x(4,i+1)=x(4,i)+Ts.*dx(4,i);
    x(5,i+1)=x(5,i)+Ts.*dx(5,i);
    x(6,i+1)=x(6,i)+Ts.*dx(6,i);
    
end

figure(1)
plot(x(1,:),x(2,:))
