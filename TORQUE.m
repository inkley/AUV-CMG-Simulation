function [tau_cmg] = TORQUE(gyro,T_OUT,Y_OUT)
% TORQUE.m
% This script calculates the Control Moment Gyroscope (CMG) torques acting
% on a neutrally buoyant underwater vehicle. The torques are computed based
% on the vehicle's angular velocities, gyroscope properties, and the CMG
% deflection angle. The calculations account for the dynamics of the CMG,
% including the angular velocity and its derivative. The torques are
% returned as a structure with components K (roll), M (pitch), and N (yaw).
% 
% Inputs
% - gyro: Structure containing the gyroscope's properties, including
% inertia.
% - T_OUT: Time vector output from the ODE solver. 
% - Y_OUT: State vector output from the ODE solver, including angular
% velocities and CMG states.
% 
% Outputs 
% - tau_cmg: Structure containing the calculated torques (K, M, N) in N*m.

%% VARIABLES
I           = gyro.I;
alpha       = Y_OUT(:,13);
Omega       = Y_OUT(:,14);   

% Derivative calcs 
del_alpha   = diff(alpha);
del_Omega   = diff(Omega);
del_t       = diff(T_OUT);

alphadot    = del_alpha./del_t;
Omegadot    = del_Omega./del_t; 

% Time midpoint for interpolation
t_mid       = T_OUT(1:end-1) + del_t/2;

% Interpolating states at original time points
p           = interp1(t_mid, Y_OUT(1:end-1,10), T_OUT, 'linear', 'extrap');
q           = interp1(t_mid, Y_OUT(1:end-1,11), T_OUT, 'linear', 'extrap');
r           = interp1(t_mid, Y_OUT(1:end-1,12), T_OUT, 'linear', 'extrap');
alpha       = interp1(t_mid, Y_OUT(1:end-1,13), T_OUT, 'linear', 'extrap');
Omega       = interp1(t_mid, Y_OUT(1:end-1,14), T_OUT, 'linear', 'extrap');

% Redefine states to match lengths
p           = p(1:end-1);
q           = q(1:end-1);
r           = r(1:end-1);
alpha       = alpha(1:end-1);
Omega       = Omega(1:end-1);

%% CMG RIGID BODY DYNAMICS
tau_cmg.K   = -I.*(cos(alpha).*Omega.*alphadot + sin(alpha).*Omegadot + cos(alpha).*Omega.*r);
tau_cmg.M   = -I.*(sin(alpha).*Omega.*alphadot - cos(alpha).*Omegadot + sin(alpha).*Omega.*r);
tau_cmg.N   = I.*Omega.*(cos(alpha).*p + sin(alpha).*q);
end