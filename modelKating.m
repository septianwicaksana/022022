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
Ts = 0.01;
t = 0:Ts:10-0.001;
x=zeros(6,1);
dx=zeros(6,1);


for i=1:2658
  
    u1=Wwheel(i)+1e-6;
    u2=SteerAngle(i);
    u=[u1 u2];

    dx = vehicleCT0Kating(x(:,i),u);
    
    x(1,i+1)=x(1,i)+Ts.*dx(1);
    x(2,i+1)=x(2,i)+Ts.*dx(2);
    x(3,i+1)=x(3,i)+Ts.*dx(3);
    x(4,i+1)=x(4,i)+Ts.*dx(4);
    x(5,i+1)=x(5,i)+Ts.*dx(5);
    x(6,i+1)=x(6,i)+Ts.*dx(6);
    
end

figure(1)
plot(x(1,:),x(2,:))
