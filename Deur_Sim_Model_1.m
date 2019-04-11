
d_theta = 0; % change in angle
d_t = 0.0005; % change in time
% v_rx = r * wheelAng - v * cos(alpha); % relative x velocity
% v_ry = v * sin(alpha); % relative y velocity
% v_r = sqrt(v_rx * v_rx + v_ry * v_ry); % relative velocity
L = 0.25;  % m ;  the length of the contact patch
r = 0.25; % m ;  effective tire radius at free rolling
v_0 = 20; % m/s ; initial velocity
v = v_0; % velocity of wheel ; initialize to initial velocity
N = 20; % number of bristles (may need to be increased)
sig0 = 181.54; % 1/m ; stiffness coefficient
sig1 = 1 % 1 / m ; rubber longitudinal lumped damping
sig2 = 0.0018; % s/m ; viscous relative damping

wheelAng = v / r;  % wheel angular velocity
v_r = r * wheelAng - v; % relative velocity
Z = zeros(1, N);

% Deflection variables (z)
for i = 2:N
    Z(i) = v - r * abs(wheelAng) * ((N - 1) / L) * (Z(i) - Z(i-1)); % v should be v_r
    disp(Z(i))
end
avgZ = mean(Z);

%gV = MUc + ((MUs - MUc) * exp(- abs(vr / vs)^ alpha));
%avgZPrime = v - (sig0 * abs(v) / gV) * avgZ;

    
    
    
    
    
    