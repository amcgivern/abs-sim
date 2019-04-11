%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Adapted from Pankaj Sharma et al. 
% Analysis of Automotive Passive Suspension System
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m = 27; M = 275; Cs = 1120; Ct = 3100; Ks = 150000; Kt = 310000;

A = [0 0 0 1; 0 0 1 -1; 0 -(Ks/M) -(Cs/M) (Cs/M); -(Kt/m) (Ks/m) (Cs/m) (-(Cs+Ct)/m)];
B = [(Ct/m); -(Ct/m); ((Ct*Cs)/(m*M));[(-(Cs*Ct)/(m*m))-((Ct*Ct)/(m*m))+(Kt/m)]];
C = [1 1 0 0];     % sprung mass displacement %
D = [0];
t = 0:0.01:10;
u = 0.1 * ones(size(t));

[Y0,X0] = lsim(A,B,C,D,u,t);

C = [0 0 1 0];   % sprung mass velocity V1 % 
[Y1,X1] = lsim(A,B,C,D,u,t);

C = [1 0 0 0];   % unsprung mass displacement X2 %
[Y2,X2] = lsim(A,B,C,D,u,t);

C = [0 0 0 1];   % unsprung mass velocity V2 %
[Y3,X3] = lsim(A,B,C,D,u,t);

C = [0 1 0 0];   % suspension travel X1 - X2 %
[Y4,X4] = lsim(A,B,C,D,u,t);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Plot the values from the simulation over time
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,3,1)
plot(t,Y0)
title('Sprung mass displacement %')

subplot(2,3,2)
plot(t,Y1)
title('Sprung mass velocity V1 %')

subplot(2,3,3)
plot(t,Y2)
title('Unsprung mass displacement X2 %')

subplot(2,3,4)
plot(t,Y3)
title('Unsprung mass velocity V2 %')

subplot(2,3,5)
plot(t,Y3)
title('Suspension travel X1 - X2 %')

