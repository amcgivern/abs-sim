% New model based on Figure 6 in Josko Deur's Modeling and Analysis of
% Longitudinal Tire Dynamics
function [mu_avg,Tmax] = fig6(a,b,q_n)
 % a: rising slope of braking moment T
 % b: falling slope
 % q_n=2 if good road conditions,otherwise-bad road conditions
% no delay, no status 0 , no constant T

    p=0.5;q=[-0.0625 0.1125]; % mu(0)=0, mu(0.2)=0.1,mu(1)=0.05,bad r.cond (a), 
                 %mu rises from s=0 to s=0.2, then decreases till
                 % s=1 \wheel block\
    if (q_n==2) % good road condition
        p=5;
        q=[-0.75 1.15]; % mu(0.2)=1, mu(1)=0.4
    end   
    I=6;  %wheel moment of inertia [kgm^2]
    r=0.3; % wheel radius [m]
    v_0=100/3.6 ; %100 km/h
    w_0=v_0/r ;
    wl_0=v_0;      %initial wheel center linear velocity 100km/h and rotational velocity s^(-1)
    g=9.81;% [m/s^2]
    m=500; %vehicle mass [kg]
    s=0;v=v_0;w=w_0;wl=wl_0;
    T=0;
    t=0;  % T-braking torque
    
    sig0 = 1000 ; % 181.54; % 1/m ; stiffness coefficient
    sig1 = 1; % 1 / m ; rubber longitudinal lumped damping
    sig2 =  0; %0.0018; % s/m ; viscous relative damping

     status=1;
   t_app_del=0; %delay in braking moment application
    t_rel_del=0; %delay in brakimng moment release
    ta_start=0;
    tr_start=0;
    t=0;
    i=0;
    dt=0.01;
    vx1=0;
    vx2=0;
    vx3=0;
     x0=0; 
    x=0;
    dx=0;
   vx=0;   
   cyc=0;
   
       int_T_dt=[0 0]; %The first stores integral of T dt and second one stores the previous value of T
    int_mu_dt=[0 0]; %The first stores integral of mu dt and second one stores the previous value of mu
    int_vx_dt=[0,0];
       %x_arr=(0);   % -----------------------------
    %vx_arr=[0]; %-----------------------------
    s_arr=[0];
   mu_arr=[0];
    t_arr=[0];
    T_arr=[0];
    v_arr=[v_0];
    w_arr=[w_0];
    wl_arr=[wl_0];
    st_arr=[0];
    k=1.0e6; % wsp sprezystosci 
    


T = 0.00051; % Torque in Nm
%T0 = 0.00125; 
T0 = sig1 / sig0;
cyc = 0;
   while cyc<20  %To avoid infinite loops
   mu= (s<=0.2)*p*s + (s>0.2)*q*[s;1]; %Calculation of mu coefficient The logic %operates "<=" and ">" %outputs 1 or 0 which can replace if condition easily
        if(status==1)
            T=T+ a*dt; %In case status 1, increase T
        end
        if(status==-1)
            
            T=T - b*dt; %In case status -1, decrease T
        end
         %In case status is neither 1 nor -1, status should be 0, in that case leave T as it is
        int_mu_dt(1) = int_mu_dt(1) + dt*(mu+int_mu_dt(2))/2; %Trapezoidal integration
        int_mu_dt(2)=mu; %Updating values
        int_T_dt(1) = int_T_dt(1) + dt*(T+int_T_dt(2))/2; %Trapezoidal integration
        int_T_dt(2)=T;          
 

cyc = cyc + 1;
        syms('sT0','sKw', "sMw", "sKv" ,"sT");
      %  f = laplace(sT0 +1) / laplace((1/sKw) * sMw * sT) + laplace((sKv * sMw * (1/sMw * sKw) * sT0 + sT0) + 1);
        d=100; %
        k=1e5; % tire stiffness
        M=0.5;
        mWheel = 3.75; % kilograms ; mass of wheel
       % Kw = 120; % wheel stiffness
       Kw = 120; 
       Kv = 50; % 
        
       % f = subs(f, [sT0, sKw, sMw, sKv], [T0, Kw, mWheel, Kv]);
       
        % Based on Fig. 6 from Deur et. al
        % Transfer function between the driving force Fw and the 
        % tire force F
        if (Kv * mWheel) / (Kw * m) > 1
            disp("Coefficients do not satisfy realistic assumption equation in Fig 8, please reevaluate ")
        end
        sys = tf([1 * T0 * (T/r), 1],[(Kw*mWheel*T)^2, Kv * mWheel *1/m* Kw * T0 *T0, 1]) % G(p) = F(p) approx= p(Fw) * (p(T0 +1) / (p^2(1/ Kw * mWheel * T) + Kv * mWHeel * (1/(mWheel * Kw * T0 * T0)) + 1))      
        
        % sys = tf( m * g , [M d k]); % transfer function: X(p) / mu(p) = m * g / (M * p ^ 2 + d * p + k)
        w0 = sqrt(Kw / (mWheel * T)); % natural frequency ; Hz
        zeta = (1/2) * ((1/T * w0) + T0 * w0); % damping ratio 
        
            if(status==1 && s > 0.21)        %If torque is increasing and s>0.22, Set status to be constant i.e. 0
            status=0;
            tr_start=t;
    end      
     
        if(status==-1 && s < 0.19) %If torque is decreasing and s<0.18, Set status to be constant i.e. 0
            status=0;
            ta_start=t;
        end
% use piecewise status function ????
         if( status==0 && t > (tr_start + t_rel_del) && s > 0.21) %If the torque is constant, release delay time has passed and s>0.22, decrease torque   
            status=-1;
         end
       
        if(status==0 && t > (ta_start + t_app_del) && s < 0.19) %If the torque is constant, application delay time has passed and s<0.18, increase torque    
                           
            status=1;
            cyc=cyc+1;
           
        end
 
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
        
%vx_arr(end+1)=vx;
%x_arr(end+1)=x;
        s_arr(end+1)=s;
        mu_arr(end+1)=mu;
        t_arr(end+1)=t+dt;
        T_arr(end+1)=T;
        v_arr(end+1)=v;
        w_arr(end+1)=w;
        wl_arr(end+1)=wl;
        st_arr(end+1)=status;
        i=i+1;
        x0=x;  % old value of tire deformation for new x calculation 
        cyc=cyc+1;   
        t=t+dt;
    
   end
    
         u=T_arr'    

        [F, t_arr] = lsim(sys,u,t_arr); % input: u = T_arr', output: tire force F
%        [yT, T_arr] = lsim(sys,T_arr,t_arr); % input: u = mu_arr, output: tire vibrations y

        plot(t_arr, F, 'r-');title("Tire Force F over Time");xlabel('Time (t) ');ylabel('Tire force (F) ')
 %       plot(T_arr, yT, 'r-');title("Tire Vibrations y vs Torque");xlabel('T');ylabel('y ')