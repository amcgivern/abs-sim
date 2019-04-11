sys = tf(massTot * g / M* p * p + d * p + k); % transfer function
F = (T0 * p + 1) / (1/Kw * massWheel * T * p * p + (p(Kv * massWheel * 1/massTot * 1/Kw * T0 + T0) + 1));
[y,t_arr] = lsim(sys,u,t_arr);
% y = tire vibrations
plot(t_arr,y);
function [mu_avg,Tmax] = Deur_sim(a,b,q_n)
% a: rising slope of braking moment T, b: falling slope, q_n=2 if good road conditions,otherwise-bad road conditions
% no delay, no status 0 , no constant T
p=0.5;q=[-0.0625 0.1125]; % mu(0)=0, mu(0.2)=0.1,mu(1)=0.05,bad r.cond (a),
%mu rises from s=0 to s=0.2, then decreases till
% s=1 \wheel block\
if (q_n == 2) % good road condition
    p=5;
    q=[-0.75 1.15]; % mu(0.2)=1, mu(1)=0.4
end

%pause on;
I=6; %wheel moment of inertia [kgm^2]
r=0.3; % wheel radius [m]
v_0=100/3.6 % initial velocity
w_0=v_0/r ; %
wl_0=v_0 %initial wheel center linear velocity 100km/h and rotational velocity s^(-1)
g=9.81; % [m/s^2]
m=150; %vehicle mass [kg]

T0 = 0.00125; % seconds
T = 0.00051; % seconds
massWheel = 3.75; % kg
massTot = 500;  % kg
Kw = 120;
Kv = 120;

% (Canudas, 2002) 
sig0 = 181.54; % 1/m ; stiffness coefficient
sig1 = 1 % 1 / m ; rubber longitudinal lumped damping
sig2 = 0.0018; % s/m ; viscous relative damping
MUc = 0.8;  % - ; normalized Coulomb friction
MUs = 1.55; % - ; normalized static friction
vs = 6.57 % m/s ; Stribeck relative velocity

dt=0.001; % change in time step over simulation

s=0;v=v_0;w=w_0;wl=wl_0;T=0;t=0;  % T-braking torque

int_T_dt=[0 0]; %The first stores integral of T dt and second one stores the previous value of T
int_mu_dt=[0 0]; %The first stores integral of mu dt and second one stores the previous value of mu
%mu - traction coefficient mu(s), s- slip
x_arr=[0];   % -----------------------------
vx_arr=[0]; %-----------------------------
s_arr=[0];
mu_arr=[0];
t_arr=[0];
T_arr=[0];
v_arr=[v_0];
w_arr=[w_0];
wl_arr=[wl_0];
st_arr=[0];
%i=0;
%t=0;
k=1.0e-5;   % tire stiffness
x0=0; %-------------------------------------------
x=0;
vx=0; %-------------------------------------------
cyc=1;
status=1;
t_app_del=0; %delay in braking moment application
t_rel_del=0; %delay in brakimng moment release
ta_start=0;
tr_start=0;
disp(t);
t=0;
i=0;
alpha = 0.5;
% alpha = 0.5;

%   ******************* loop ************************************

%^while i<5
while cyc<2000  %To avoid infinite loops
    t=t+dt ;    %updating time in each loop
    % disp(t);
    mu= (s<=0.2)*p*s + (s>0.2)*q*[s;1]; %Calculation of mu coefficient The logic operates "<=" and ">"
    %outputs 1 or 0 which can replace if condition easily

    
    if(status==1)
        T=T+ a*dt; %In case status 1, increase T
    end
    
    if(status==-1)
        
        T=T - b*dt; %In case status -1, decrease T
    end
    %In case status is neither 1 nor -1, status should be 0, in that case leave T as it is

    %x=k*(m*g*mu+T/r);  %    tire deformation1/k(mgmu+T/r)----------------------------------
    x=k*(m*g*mu);  %    tire deformation1/k(mgmu)---------------------------------
    int_mu_dt(1) = int_mu_dt(1) + dt*(mu+int_mu_dt(2))/2; %Trapezoidal integration
    int_mu_dt(2)=mu; %Updating values
    int_T_dt(1) = int_T_dt(1) + dt*(T+int_T_dt(2))/2; %Trapezoidal integration
    int_T_dt(2)=T;                                                          %Updating values
    
    v = -1*g*int_mu_dt(1) + v_0;                % linear movement eq:  m*dv/dt=(m*g)*mu   integration
  %  w = (1/I)*(m*g*r*int_mu_dt(1) - int_T_dt(1)) + w_0 ;%  rotational movement eq: %  I(dw/dt)=(mg)r(mu)-T   integration
   % z = (sign(v) * r * w) - abs(v) / abs(v); % the internal friction state ; zeta1 - zeta0 / zeta0
   s = (v - r*w+vx)/v;                            %slip
    vr = (r * w) - v; % m/s ; relative velocity

   gVr = MUc + ((MUs - MUc) * exp(- abs(vr / vs)^ alpha));
   zPrime = vr - ((sig0 * abs(vr) / gVr)* s);
  %zPrime = abs(vr) * 
   w = ((1/I)*(m*g*r*int_mu_dt(1) - int_T_dt(1)) + w_0) * (sig0 * s + sig1 * zPrime + sig2 * vr);  %  rotational movement eq: %  I(dw/dt)=(mg)r(mu)-T   integration  
  vx=(x-x0)/dt ;
    
    %s = (v - r*w)/v;                            %slip
    
    wl=w*r     ;                              %linear speed of wheel core
    
    if(status==1 && s > 0.21)        %If torque is increasing and s>0.22, Set status to be constant i.e. 0
        status=0;
        tr_start=t;
    end
    
    if(status==-1 && s < 0.19) %If torque is decreasing and s<0.18, Set status to be constant i.e. 0
        status=0;
        ta_start=t;
    end
    
    if( status==0 && t > (tr_start + t_rel_del) && s > 0.21) %If the torque is constant, release delay time has passed and s>0.22, decrease torque
        status=-1;
    end
    
    if(status==0 && t > (ta_start + t_app_del) && s < 0.19) %If the torque is constant, application delay time has passed and s<0.18, increase torque
        
        status=1;
    
    end
    
    cyc=cyc+1;
    
    if(v<=2)  %If velocity less than 0, stop
        break
    end
    
    if(s<0)
        s=0;
    end
    
    if(T<0)
        T=0;
    end
    
    if(T==0 && s==0)
        status=1;
    end
    
    if(s > 0.99)
        break
    end
    vx_arr(end+1)=vx;
    x_arr(end+1)=x;
    s_arr(end+1)=s;
    mu_arr(end+1)=mu;
    t_arr(end+1)=t;
    T_arr(end+1)=T;
    v_arr(end+1)=v;
    w_arr(end+1)=w;
    wl_arr(end+1)=wl;
    st_arr(end+1)=status;
    i=i+1;
    x0=x;  % old value of tire deformation for new x calculation
end
mu_avg=mean(mu_arr);
Tmax=max(T_arr);

subplot(2,2,1);plot(t_arr,v_arr,'-', t_arr,wl_arr, 'b-' , t_arr,vx_arr, '-',  'LineWidth',0.5);title('v and wl  and vx vs t');xlabel('t');ylabel('w and V');
%subplot(2,2,1);plot(t_arr,vx_arr,'-', t_arr,x_arr,'b-','    LineWidth',0.5);title('vx and x vs t');xlabel('t');ylabel('x and Vx');
%subplot(2,2,1);plot(t_arr,vx_arr,'b-', t_arr,v_arr,'-',     'LineWidth',0.5);title(' vx and v vs t');xlabel('t');ylabel('x');
subplot(2,2,2);plot(t_arr,T_arr,'-');                                   title('T  vs t');xlabel('t');ylabel('T');
%subplot(2,2,3);plot(t_arr,v_arr,'-');title('v vs t');xlabel('t');ylabel('v');
subplot(2,2,3);plot(t_arr,s_arr,'-',t_arr,mu_arr,'-')     ;title('s and mu vs t');xlabel('t');ylabel('s mu');
subplot(2,2,4);plot(t_arr,st_arr,'-','LineWidth',1);title('T control vs t');xlabel('t');ylabel('-1  0  +1');

end

