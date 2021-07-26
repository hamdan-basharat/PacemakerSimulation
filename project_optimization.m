%Optimization of the system using IAE error
clear variables;
close all;
clc;

%P = [0.5524 0.6024 0.6524 0.7024 0.7524];
%I = [1.5388 1.5888 1.6388 1.6888 1.7388];
%D = [0.0525 0.1025 0.1525 0.2025 0.2525];
runtimestart = clock; %measure runtime
disp("started at:");
disp(runtimestart);

%create range of PID gains
P = 0.4 : 0.01 : 0.8;
I = 0 : 0.01 : 0.4;
D = 0.1525;

%initialize array to store all PID values, and their associated errors
%(IAe, ISe...)
Error_array = zeros(1,7);

set_param('project_optimization_sim/PID Controller','D',string(D));%hard set derivative gain
for j = 1:length(P)
    
    set_param('project_optimization_sim/PID Controller','P',string(P(j)));
    for k = 1:length(I)
            
            set_param('project_optimization_sim/PID Controller','I',string(I(k)));
            
            out = sim('project_optimization_sim',30); %create simulation
            
            %calculating all error types 
            epsilon = abs(75 - y_PI);
            epsilon2 = epsilon.^2;
            teps = epsilon.*t;
            teps2 = epsilon2.*t;
            
            % Use error and trapezoid rule for integration to determine the
            % integral error
            IAE = 0;
            ISE = 0;
            ITAE = 0;
            ITSE = 0;
            for i = 1:length(t)-1
                IAE(i+1) = IAE(i) + 0.5*(epsilon(i) + epsilon(i+1))*(t(i+1)-t(i)); 
                ISE(i+1) = ISE(i) + 0.5*(epsilon2(i) + epsilon2(i+1))*(t(i+1)-t(i)); 
                ITAE(i+1) = ITAE(i) + 0.5*(teps(i) + teps(i+1))*(t(i+1)-t(i)); 
                ITSE(i+1) = ITSE(i) + 0.5*(teps2(i) + teps2(i+1))*(t(i+1)-t(i)); 
            end
            
            %store the errors, and associated PID values used to calculate
            %it
            Error_array = [Error_array ; P(j) I(k) D IAE(length(t)-1) ISE(length(t)-1) ITAE(length(t)-1) ITSE(length(t)-1)];
            %IAE_array(m,1:length(IAE)) = IAE;
            
        
    end
end

runtimeend=clock;
disp("Ended at: ");
disp(runtimeend);

%%
Error_array(1,:)=[];
%{
figure;
scatter3(Error_array(:,1),Error_array(:,2),Error_array(:,4));
xlabel('P Value');
ylabel('I Value)');
zlabel('IAE Error');
title('IAE Value to Variable P and I ')
%}
%{

figure;
scatter3(Error_array(:,1),Error_array(:,2),Error_array(:,5));
xlabel('P Value');
ylabel('I Value)');
zlabel('ISE Error');
title('ISE Value to Variable P and I ')
%}

%{
figure;
scatter3(Error_array(:,1),Error_array(:,2),Error_array(:,6));
xlabel('P Value');
ylabel('I Value)');
zlabel('ITAE Error');
title('ITAE Value to Variable P and I ')
%}

%{
figure(1);
stem3(Error_array(:,1),Error_array(:,2),Error_array(:,7));
xlabel('P Value');
ylabel('I Value)');
zlabel('ITSE Error');
title('ITSE Value to Variable P and I ')

%}

%plot a surface of the IAE error
figure(2)
x=Error_array(:,1);
y=Error_array(:,2);
z = Error_array(:,4);
xv = linspace(min(x), max(x), 41);
yv = linspace(min(y), max(y), 41);
[X,Y] = meshgrid(xv, yv);
Z = griddata(x,y,z,X,Y);

surf(X,Y,Z);
xlabel('P Value');
ylabel('I Value');
zlabel('IAE Error');
title('IAE Value Using Variable Proportional and Integral Gain')
grid on
shading interp


