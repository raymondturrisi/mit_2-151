clc;
clear all;
close all;

% Define force equation from Kang paper
syms Pg D_o q E a_o
F_nl=Pg*D_o^2*pi/4*(3*(1-q*E)^2/tan(a_o)^2-1/(sin(a_o)^2));

% Strain definition
syms L L_o
E=(L_o-L)/L_o;

% Correction factor from Kang paper
syms cq1 cq2
q= 1+ cq1*exp(cq2*Pg);


F_nl=subs(F_nl);

% Take partial derivatives
dF_dPg= diff(F_nl,Pg);
dF_dL=diff(F_nl,L);

% Evaluate partials at operating point
syms L_i Pg_i
L = L_i;
Pg = Pg_i;
dF_dPg = subs(dF_dPg);
dF_dL=subs(dF_dL);

% Evaluate initial force
F_i=subs(F_nl);

syms Pg L
dF= dF_dPg*Pg + dF_dL*L; 


% Evaluate for system constants
D_o = 0.010; %meters (from datasheet)
a_o = 23*pi/180; %pi/6;  %rad (GUESS)
L_o = 0.300; %meters - measured in lab
cq1= 3; % (FIT FROM DATA)
cq2= -0.000002;  %(FIT FROM DATA)

% Store force equations in global variables to be used in ODE solving
global linear_force nonlinear_force
linear_force = subs(dF);
nonlinear_force = subs(F_nl);


L_i = .9625*L_o; %meters (10% strain)
Pg_i = 225e3; %Pa



%Coefficients for state equations
L_coeff=double(diff(subs(dF),L));
Pg_coeff=double(diff(subs(dF),Pg));
F_i=subs(F_i);

global m k b tau
% System parameters
m=4.4; %kg
k = 0; %N/m
zeta=0.5;
b=2*zeta*sqrt((k+L_coeff)*m);%N*s/m
tau=.2; %Pressure lowpass time constant (sec)

%State equations
A=[0 1
    (-k-L_coeff)/m -b/m];
B=[0 ;
    -Pg_coeff/m];

C=[1 0];
D=0;

%


sys=ss(A,B,C,D);
%%


[y_s,t_s]=step(sys,5);
step_size = 2.25; %bar
step_time = 10; % sec

% figure()
% plot(t_s,y_s)

time_step =t_s+step_time;
L_step=123+step_size*1e8*y_s;


%%
load('20221109_testing.mat');
load('20221116_testing.mat');

%% Step response from 2.25 bar to 4.5 bar
set=sb_step_10_16;
time = set.py1Time; %sec
pressure_input = (set.py1Val-4000)*6/16000; %bar (I think)
position =(125/16)*(set.zx1Val-4)+25; %mm
pressure_reg = 6/(16)*(set.px2Val-4); %bar (I think)
pressure_compr = 12/16*(set.px1Val-4); %bar (I think)
load_cell = 9.81*(set.lc1Val/1000); %N


%Calculate analytical step response
t_dur=5; %sec
t_step = 0.001; %sec
t_offset = 10; %sec
L_init = L_meas_to_real(123.76); %m
Pg_init = 2.25*1e5; %Pa
Pg_final = 4.5*1e5; %Pa

%Linear model
l_nl='l';
lp=0;
[t_l,y_l,f_l]= step_response(t_offset,t_dur,t_step,L_init,Pg_init,Pg_final,l_nl,lp);
y_l=L_real_to_meas(y_l);

%Nonlinear model
l_nl='nl';
[t_nl,y_nl,f_nl]= step_response(t_offset, t_dur,t_step,L_init,Pg_init,Pg_final,l_nl,lp);
y_nl=L_real_to_meas(y_nl);

%Added lowpass
l_nl='l';
lp=1;
[t_l_lp,y_l_lp,f_l_lp]= step_response(t_offset,t_dur,t_step,L_init,Pg_init,Pg_final,l_nl,lp);
y_l_lp=L_real_to_meas(y_l_lp);
l_nl='nl';
[t_nl_lp,y_nl_lp,f_nl_lp]= step_response(t_offset, t_dur,t_step,L_init,Pg_init,Pg_final,l_nl,lp);
y_nl_lp=L_real_to_meas(y_nl_lp);



% Plot data
figure(1);
subplot(3,1,1);
plot(time,pressure_input);
xlabel('Time [s]')
title('Pressure step input')
ylabel('Pressure (Bar)')

subplot(3,1,2);
plot(time,position);
xlabel('Time [s]');
title('Position sensor')
ylabel('Position [mm]');


subplot(3,1,3);
plot(time,load_cell);
title('Load cell')
xlabel('Time [s]');
ylabel('Load [N]');



figure(2);
subplot(3,1,1);
plot(time-10,pressure_input);
xlabel('Time [s]')
title('Pressure step input')
ylabel('Pressure (Bar)')
xlim([9.5-10,13-10])
grid on;

subplot(3,1,2);
plot(time-10,position,'.-');
xlabel('Time [s]');
title('Actuator length for step in pressure')
ylabel('Length [mm]');
hold on;
plot(t_l-10,y_l(:,1));
plot(t_nl-10,y_nl(:,1));
plot(t_l_lp-10,y_l_lp(:,1));
plot(t_nl_lp-10,y_nl_lp(:,1));
legend('Experiment','Linear model','Nonlinear model','Linear model (LP)','Nonlinear model (LP)');
xlim([9.5-10,13-10]) 
grid on;

subplot(3,1,3);
plot(time-10,load_cell,'.-');
title('Actuator force for step in pressure')
xlabel('Time [s]');
ylabel('Load [N]');
xlim([9.5-10,13-10])
grid on;
hold on;
plot(t_l-10,f_l);
plot(t_nl-10,f_nl);
plot(t_l_lp-10,f_l_lp);
plot(t_nl_lp-10,f_nl_lp);
legend('Experiment','Linear model','Nonlinear model','Linear model (LP)','Nonlinear model (LP)');


figure(5)

%% Step response from 1.5 bar to 5.25 bar
set=sb_step_12_14;
time = set.py1Time; %sec
pressure_input = (set.py1Val-4000)*6/16000; %bar (I think)
position =(125/16)*(set.zx1Val-4)+25; %mm
pressure_reg = 6/(16)*(set.px2Val-4); %bar (I think)
pressure_compr = 12/16*(set.px1Val-4); %bar (I think)
load_cell = 9.81*(set.lc1Val/1000); %N


%Calculate analytical step response
t_dur=5; %sec
t_step = 0.001; %sec
t_offset = 10; %sec
L_init = L_meas_to_real(122.3); %m
Pg_init = 3*1e5; %Pa
Pg_final = 4.5*1e5; %Pa

%Linear model
l_nl='l';
lp=0;
[t_l,y_l,f_l]= step_response(t_offset,t_dur,t_step,L_init,Pg_init,Pg_final,l_nl,lp);
y_l=L_real_to_meas(y_l);

%Nonlinear model
l_nl='nl';
[t_nl,y_nl,f_nl]= step_response(t_offset, t_dur,t_step,L_init,Pg_init,Pg_final,l_nl,lp);
y_nl=L_real_to_meas(y_nl);

%Added lowpass
l_nl='l';
lp=1;
[t_l_lp,y_l_lp,f_l_lp]= step_response(t_offset,t_dur,t_step,L_init,Pg_init,Pg_final,l_nl,lp);
y_l_lp=L_real_to_meas(y_l_lp);
l_nl='nl';
[t_nl_lp,y_nl_lp,f_nl_lp]= step_response(t_offset, t_dur,t_step,L_init,Pg_init,Pg_final,l_nl,lp);
y_nl_lp=L_real_to_meas(y_nl_lp);



% Plot data
figure(3);
subplot(3,1,1);
plot(time,pressure_input);
xlabel('Time [s]')
title('Pressure step input')
ylabel('Pressure (Bar)')

subplot(3,1,2);
plot(time,position);
xlabel('Time [s]');
title('Position sensor')
ylabel('Position [mm]');


subplot(3,1,3);
plot(time,load_cell);
title('Load cell')
xlabel('Time [s]');
ylabel('Load [N]');



figure(4);
subplot(3,1,1);
plot(time,pressure_input);
xlabel('Time [s]')
title('Pressure step input')
ylabel('Pressure (Bar)')
xlim([9.5,14])
grid on;

subplot(3,1,2);
plot(time,position,'.-');
xlabel('Time [s]');
title('Position sensor')
ylabel('Position [mm]');
hold on;
plot(t_l,y_l(:,1));
plot(t_nl,y_nl(:,1));
plot(t_l_lp,y_l_lp(:,1));
plot(t_nl_lp,y_nl_lp(:,1));
legend('Experiment','Linear model','Nonlinear model','Linear model (LP)','Nonlinear model (LP)');
xlim([9.5,14]) 
grid on;

subplot(3,1,3);
plot(time,load_cell,'.-');
title('Load cell')
xlabel('Time [s]');
ylabel('Load [N]');
xlim([9.5,14])
grid on;
hold on;
plot(t_l,f_l);
plot(t_nl,f_nl);
plot(t_l_lp,f_l_lp);
plot(t_nl_lp,f_nl_lp);
legend('Experiment','Linear model','Nonlinear model','Linear model (LP)','Nonlinear model (LP)');



function xdot = sys_l_lp(~,x)
    global m k b L_i Pg_i Pg_f tau

    % State extraction
    L=x(1);
    L_dot=x(2);
    Pg=x(3);

    % Force calculation
    f=force_l(L,Pg,L_i,Pg_i);

    % State equations
    x1dot = L_dot;
    x2dot = -k/m*L -b/m*L_dot -(1/m)*f;
    x3dot = 1/tau*(Pg_f-Pg);
    

    % Return the state derivatives
    xdot = [x1dot;
            x2dot;
            x3dot];

end

function xdot = sys_nl_lp(~,x)
    global m k b L_i Pg_i Pg_f tau

    % State extraction
    L=x(1);
    L_dot=x(2);
    Pg=x(3);

    % Force calculation
    f=force_nl(L,Pg);

    % State equations
    x1dot = L_dot;
    x2dot = -k/m*L -b/m*L_dot -(1/m)*f;
    x3dot = 1/tau*(Pg_f-Pg);
    

    % Return the state derivatives
    xdot = [x1dot;
            x2dot;
            x3dot];

end

function xdot = sys_l(~,x)
    global m k b L_i Pg_i Pg_f

    % State extraction
    L=x(1);
    L_dot=x(2);

    % Force calculation
    f=force_l(L,Pg_f,L_i,Pg_i);

    % State equations
    x1dot = L_dot;
    x2dot = -k/m*L -b/m*L_dot -(1/m)*f;
    

    % Return the state derivatives
    xdot = [x1dot;
            x2dot];

end

function xdot = sys_nl(~,x)
    global m k b Pg_f

    % State extraction
    L=x(1);
    L_dot=x(2);

    % Force calculation
    f=force_nl(L,Pg_f);

    % State equations
    x1dot = L_dot;
    x2dot = -k/m*L -b/m*L_dot -f/m;
    

    % Return the state derivatives
    xdot = [x1dot;
            x2dot];
end

function [t,x,f] = step_response(t_start, t_dur,t_step,L_init,Pg_init,Pg_final,l_nl,lp)
    %     t_start - Starting time of simulation [s]
    %     t_dur - Test duration [s]
    %     t_step - ODE solver step size [s]
    %     L_init - Initial actuator length [m]
    %     Pg_init - Initial system pressure [Pa]
    %     Pg_final - Final system pressure [Pa]
    %     l_nl - 'l' means linear, 'nl' means nonlinear
    %     lp - low pass filter? (1 or 0)
    
    global L_i Pg_i Pg_f
    L_i=L_init;
    Pg_i=Pg_init;
    Pg_f=Pg_final;
    

    %Time vector
    tvec=linspace(t_start,t_start+t_dur,floor(t_dur/t_step));
    
    %Initial conditions (assuming zero initial velocity)
    

     % Use ode45 to integrate the state equations
    if(l_nl=='l')
        if(lp==1)
            x0=[0;
                0;
                Pg_init];
            [t, x] = ode45(@sys_l_lp, tvec, x0);
            f=force_l(x(:,1),x(:,3),L_init,Pg_init);
        else
             x0=[0;
                 0];
             [t, x] = ode45(@sys_l, tvec, x0);
             f=force_l(x(:,1),Pg_final,L_init,Pg_init);
        end

        % Add original length back in
        x(:,1)=x(:,1)+L_i;

    elseif(l_nl=='nl')
        if(lp==1)
             x0=[L_i;
                0;
                Pg_init];
            [t, x] = ode45(@sys_nl_lp, tvec, x0);
            f=force_nl(x(:,1),x(:,3));
        else
            x0=[L_i;
                0];
            [t, x] = ode45(@sys_nl, tvec, x0);
            f=force_nl(x(:,1),Pg_final);
        end
        
    else
        sprintf('Neither linear or nonlinear chosen');
    end

    global m
    f=f+m*9.81;
    
end

function f = force_l(L,Pg,L_i,Pg_i)
    global linear_force
    
    Pg=Pg-Pg_i;
    f=double(subs(linear_force));
end

function f = force_nl(L,Pg)
    global nonlinear_force
    
    f=double(subs(nonlinear_force));
end

function L_real = L_meas_to_real(L_meas)
    % Input : measured L (mm)
    % Output: real L (m)
    
    % Max LVDT length measured (135mm)
    % Assuming max actuator length real = 0.3m
    
    L_real = 0.3-(135-L_meas)/1000;

end

function L_meas = L_real_to_meas(L_real)
    % Input: real L (m)
    % Output : measured L (mm)
    
    % Max LVDT length measured (135mm)
    % Assuming max actuator length real = 0.3m

    L_meas=135-(0.3-L_real)*1000;
end