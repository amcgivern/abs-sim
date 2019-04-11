clear all
close all
clc


% System parameters
mwe = 1;
m = 200;
T = 0.01;
T0 = 0.004;
Kw = 100;
Kv = 0.5;
r = 0.2; % Wheel rayon


Tf = 0.5; % Final time
sim('OneWheelVehicleModel.mdl',Tf)

% Draw results

figure
plot(Fw(:,1),Fw(:,2),F(:,1),F(:,2),'linewidth',1)
grid on
xlim([0 Tf])
set(gca,'FontSize',8)
ylabel('Driving-braking force [N]','Interpreter','latex')
xlabel('Time [s]','Interpreter','latex')
legend('$F_\omega$','$F$','location','southeast','Interpreter','latex','EdgeColor',[0.94,0.94,0.94])

figure
plot(V(:,1),V(:,2))
grid on
xlim([0 Tf])
set(gca,'FontSize',8)
ylabel('Car speed [m/s]','Interpreter','latex')
xlabel('Time [s]','Interpreter','latex')

figure
plot(Vw(:,1),Vw(:,2))
grid on
xlim([0 Tf])
set(gca,'FontSize',8)
ylabel('Wheel angular Velocity [rad/s]','Interpreter','latex')
xlabel('Time [s]','Interpreter','latex')


figure  
for i=1:length(V)
    if Vw(i,2)==0
        slip(i)=0;
    else
        slip=(r*Vw(:,2)-V(:,2))./(r*Vw(:,2));
    end
end
plot(Vw(:,1),slip)
grid on
xlim([0 Tf])
set(gca,'FontSize',8)
ylabel('Wheel angular Velocity [rad/s]','Interpreter','latex')
xlabel('Time [s]','Interpreter','latex')

