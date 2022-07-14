clear;clc;
x=zeros(6,1);
x0 = [1 1 1 3 2 1.5]';
wwheel = load('Ww.mat').ans;
steerAngle = load('steer.mat').ans;

% Ts = 1/265.9;
Ts = 0.01;
t = 0:Ts:10-0.001;

Xdot = zeros(6,2658);
X = zeros(6,2659);
Xdot(:,1) = x(:,1);

for i = 1:2658
    u = [wwheel(i+1) steerAngle(i+1)];
    Xdot(:,i) = vehicleCT0(x,u);
    x(1,i+1) = x(1,i)+0.01.*Xdot(1,i);
    x(2,i+1) = x(2,i)+0.01.*Xdot(2,i);
    x(3,i+1) = x(3,i)+0.01.*Xdot(3,i);
    x(4,i+1) = x(4,i)+0.01.*Xdot(4,i);
    x(5,i+1) = x(5,i)+0.01.*Xdot(5,i);
    X(6,i+1) = x(6,i)+0.01.*Xdot(6,i);

end

%%

figure(1)
plot(x(1,:),x(2,:))
% for i = 1 : 6
%     subplot(6,1,i);
%     plot(t,Xdot(i,:))
% end
