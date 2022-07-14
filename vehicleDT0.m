function xk1 = vehicleDT0(xk, uk, Ts)
%% Discrete-time nonlinear dynamic model of a pendulum on a cart at time 

M = 10;
delta = Ts/M;
xk1 = xk;
for ct=1:M
    xk1 = xk1 + delta*vehicleCT0(xk1,uk);
end
% Note that we choose the Euler method (first oder Runge-Kutta method)
% because it is more efficient for plant with non-stiff ODEs.  You can
% choose other ODE solvers such as ode23, ode45 for better accuracy or
% ode15s and ode23s for stiff ODEs.  Those solvers are available from
% MATLAB.
