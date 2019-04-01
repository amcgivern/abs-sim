% Amanda McGivern
% Reference Josko Deur: Modeling and Analysis of Longitudinal Tire Dynamics
% Based on LuGre Friction Model

T0 = 0;
omega = 1; % Angular velocity of wheel
N = 11; % Number of bristles
Fn = 500; %g N
sig0Fn = 200; % 1/m, sig0 / Fn
sig1Fn = 0.25; % (0)s/m
sig2 = 0;
L = 0.25; % m
FcFn = 0.5; % Fc/Fn
vx = 12.5; % m/s
delta2 = 1;
theta = 1;
kappa = 1.2;
m = 500; % kg;
r = 0.25; % m
J = 0.2344; % kgm^2
v0 = 20; % m/s


Fm = 0; % force of the driving/braking force
mx = J / r * r; % equivalent wheel mass
vw = r * omega; % Velocity of wheel

Km = 0; %???????????

F = T0 ;


% Simulation model Eqn 9
for i = 2:N
    %z(i) = vr - r * abs(omega) * (N - 1)/L * (z(i) - z(i - 1)); % zi == 0 ?
end

% Static Model Eqn 24
% Where:
% S = s/1-s (braking)
% S = s (traction)
% F(s) = sgn(vr) * ((((sig0 * L)/kappa) * (S / ((sig0 * L)/kappa) * (1 / g(s)) * S + 1)) + sig2 * S * r * abs(omega));

% Linearization of lumped LuGre tire model (Appendix B)
%(Tp + 1) * sig0 * deltaz;





