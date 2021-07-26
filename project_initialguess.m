clear variables;
clc;
close all;

omega = logspace(-3,3,1000);

for i = 1:length(omega)
    %Bode plot is evaluated only for GLOOP
    Gloop(i) = (1)*(0.7)*(exp(-0.145*(omega(i)*1i)))*(8/((omega(i)*1i)+8))*(169/(((omega(i)*1i))^2+ 20.8*(omega(i)*1i)));
    A(i) = norm(Gloop(i));
    phase(i) = angle(Gloop(i))*(180/pi);
    
    if phase(i) >= 0
        phase(i) = -180 - (180 - phase(i));
    end
end

for j = 1:length(phase)
    if phase(j) + 180 < 0
        index = j;
        break
    end
end

index = index - 1;

subplot(2,1,1)
loglog(omega,A,'k-','LineWidth',1)
hold on
plot([omega(index) omega(index)] , [min(min(A)), max(max(A))] ,'r-')
hold off
box on;
grid on;
ylabel('Gain (A)')

subplot(2,1,2)
semilogx(omega,phase,'k-','LineWidth',1)
hold on;
plot([min(omega) max(omega)],[-180 -180],'r-')
plot([omega(index) omega(index)] , [phase(index),0] , 'r-')
hold off
box on;
grid on;
xlabel('Input Frequency (\omega)');
ylabel('Phase Shift (\theta , \circ)');

omega_c = omega(index);
G_critical = (1)*(0.7)*(exp(-0.145*(omega_c*1i)))*(8/((omega_c*1i)+8))*(169/(((omega_c*1i))^2+ 20.8*(omega_c*1i)));
A_critical = norm(G_critical)
K_u = 1/A_critical
T_u = 2*pi()/omega_c

%PID
K_C_ZN = K_u/1.7
T_I_ZN = T_u/2.0
K_D_ZN = T_u/8

