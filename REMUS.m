function [Etadot,tau] = REMUS(~,auv,contpar,params,state,tauC,tau_cmg) 
% REMUS.m
% This script models the dynamics of the REMUS (Remote Environmental
% Monitoring UnitS) underwater vehicle. The script calculates the
% generalized forces and moments acting on the vehicle based on its current
% state, control inputs, and gyroscopic effects.
% 
% Notes
% - Cross-coupling terms related to fins have been removed or commented out.
% - Hydrostatic terms are not included, assuming a neutrally buoyant vehicle.
% - The inertia matrix is simplified to include only the diagonal
% components (Ixx, Iyy, Izz), which is a reasonable assumption for this
% simulation but may need revisiting for more complex cases.
% 
% Inputs
% - auv: Structure containing the AUV's properties, such as mass and inertia.
% - contpar: Structure containing control parameters, including angular accelerations.
% - params: Structure containing various physical parameters of the vehicle.
% - state: Current state vector of the vehicle, including position, orientation, and velocities.
% - tauC: Control torques applied to the vehicle.
% - tau_cmg: Torques generated by the Control Moment Gyroscope (CMG).
% 
% Outputs
% - Etadot: Time derivative of the generalized position vector.
% - tau: Generalized forces and moments acting on the vehicle.

%% Variable pass-throughs
m           = auv.m;

alphadot    = contpar.alphadot;
Omegadot    = contpar.Omegadot;

phi         = state(4);
theta       = state(5);
psi         = state(6);
u           = state(7);
v           = state(8);
w           = state(9);
p           = state(10);
q           = state(11);
r           = state(12);

Ix          = params.Ix;
Iy          = params.Iy;
Iz          = params.Iz;
xg          = params.xg;
yg          = params.yg;
zg          = params.zg;

%% REMUS hydrodynamic coefficients (Prestero 2001)
% Forces
Xuu         = -1.62;        %           (kg/m)
Xudot       = -0.93;        %           (kg)
Xwq         = 0;            % -35.5;    (kg/rad)
Xqq         = 0;            % -1.93;    (kgm/rad)
Xvr         = 0;            % 35.5;     (kg/rad)
Xrr         = 0;            % -1.93;    (kgm/rad)
Yvv         = -131;         %           (kg/m)
Yrr         = 0;            % 0.632;    (kgm/rad^2)
Yuv         = 0;            % -28.6;    (kg/m)
Yvdot       = -35.5;        %           (kg)
Yrdot       = 0;            % 1.93;     (kgm/rad)
Yur         = 0;            % 5.22;     (kg/rad)
Ywp         = 0;            % 35.5;     (kg/rad)
Ypq         = 0;            % 1.93;     (kgm/rad)
Zww         = -131;         %           (kg/m)
Zqq         = 0;            % -0.632;   %kgm/rad^2
Zuw         = 0;            % -28.6;    (kg/m)
Zwdot       = -35.5;        %           (kg)
Zqdot       = 0;            % -1.93;    (kgm/rad)
Zuq         = 0;            % -5.22;    (kg/rad)
Zvp         = 0;            % -35.5;    (kg/rad)
Zrp         = 0;            % 1.93;     (kg/rad)

% Moments
Kpp         = -0.0013;      %           (kgm^2/rad^2)    (should correct to remove fin drag)
Mww         = 0;            % 3.18;     (kg)
Mqq         = -9.4;         %           (kgm^2/rad^2)
Muw         = 0;            % 24.0;     (kg)
Mwdot       = 0;            % -1.93;    (kgm)
Mqdot       = -4.88;        %           (kgm^2/rad)
Muq         = 0;            % -2.0;     (kgm^2/rad)
Mvp         = 0;            % -1.93;    (kgm^2/rad)
Mrp         = 0;            % 4.86;     (kgm^2/rad^2)
Nvv         = 0;            % -3.18;    (kg)
Nrr         = -9.4;         %           (kgm^2/rad^2)
Nuv         = 0;            % -24;      (kg)
Nvdot       = 0;            % 1.93;     (kgm)
Nrdot       = -4.88;        %           (kgm^2/rad)
Nur         = 0;            % -2.0;     (kgm^2/rad)
Nwp         = 0;            % -1.93;    (kgm^2/rad)
Npq         = 0;            % -4.86;    (kgm^2/rad^2)

% Define Hydrostatic forces (=0 for neutrally bouyant vehicle)
XHS         = 0;    % -(W-B)*sin(theta);
YHS         = 0;    % (W-B)*cos(theta)*cos(phi);
ZHS         = 0;    % (W-B)*cos(theta)*cos(phi);
KHS         = 0;    % -(yg*W-yb*B)*cos(theta)*cos(phi) - (zg*W-Zb*B)*cos(theta)*sin(phi);
MHS         = 0;    % -(zg*W-zb*B)*sin(theta) - (xg*W-xb*B)*cos(theta)*cos(phi);
NHS         = 0;    % -(xg*W-xb*B)*cos(theta)*sin(phi) - (yg*W-yb*B)*sin(theta);

% Define sum of hydrodynamic forces in each DOF
Xhyd        = Xuu*u*abs(u) + Xwq*w*q + Xqq*q^2 + Xvr*v*r + Xrr*r^2;                % + Xudot*udot;
Yhyd        = Yvv*v*abs(v) + Yrr*r*abs(r) + Yur*u*r + Ywp*w*p + Ypq*p*q + Yuv*u*v; % + Yvdot*vdot + Yrdot*rdot;
Zhyd        = Zww*w*abs(w) + Zqq*q*abs(q) + Zuq*u*q + Zvp*v*p + Zrp*r*p + Zuw*u*w; % + Zwdot*wdot + Zqdot*qdot;
Khyd        = Kpp*p*abs(p);
Mhyd        = Mww*w*abs(w) + Mqq*q*abs(q) + Muq*u*q + Mvp*v*p + Mrp*r*p + Muw*u*w; % + Mwdot*wdot + Mqdot*qdot;
Nhyd        = Nvv*v*abs(v) + Nrr*r*abs(r) + Nur*u*r + Nwp*w*p + Npq*p*q + Nuv*u*v; % + Nvdot*vdot + Nrdot*rdot;

% Total forces
X           = XHS + Xhyd + tauC.XD;
Y           = YHS + Yhyd + tauC.YD;
Z           = ZHS + Zhyd + tauC.ZD;
K           = KHS + Khyd + tauC.KD + tau_cmg.K; 
M           = MHS + Mhyd + tauC.MD + tau_cmg.M;
N           = NHS + Nhyd + tauC.ND + tau_cmg.N;

tau         = [X;Y;Z;K;M;N];

Ma          = [m-Xudot, 0, 0, 0, m*zg, -m*yg;
              0, m-Yvdot, 0, m*zg, 0, m*xg-Yrdot;
              0, 0, m-Zwdot, m*yg, -m*xg-Zqdot, 0;
              0, m*zg, m*yg, Ix, 0, 0;
              m*zg, 0, -m*xg-Mwdot, 0, Iy-Mqdot, 0;
              -m*yg, m*xg-Nvdot, 0, 0, 0, Iz-Nrdot];

CRB         = [0, -m*r, m*q, m*(r*zg+q*yg), -m*q*xg, -m*r*xg;
              m*r, 0, -m*p, -m*p*yg, m*(r*zg+p*xg), -m*r*yg;
              -m*q, m*p, 0, -m*p*zg, -m*q*zg, m*(q*yg+p*xg);
              -m*(r*zg+q*yg), m*p*yg, m*p*zg, 0, -Iz*r, Iy*q;
              m*q*xg, -m*(r*zg+p*xg), m*q*zg, Iz*r, 0, -Ix*p;
              m*r*xg, m*r*yg, -m*(q*yg+p*xg), -Iy*q, Ix*p, 0];

vec_nu      = [u;v;w;p;q;r];

%% N2L in non-inertial frame gives rate of change of body fixed vector nu
nudot       = Ma\(tau - CRB*vec_nu);

% Define rotation matrices to get velocities in inertial frame
J1          = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi);
              sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi), -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi);
              -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];

J2          = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
              0, cos(phi), -sin(phi);
              0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

nu          = [u;v;w;p;q;r];
etadot      = [J1,zeros(3,3);zeros(3,3),J2]*nu;

% All together Now
Etadot      = [etadot;nudot;alphadot;Omegadot];  