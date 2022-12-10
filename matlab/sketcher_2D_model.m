clc;
clear all;
close all;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% V2 Setup system parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



l_op = 0.27; % Operating length of actuator[m]
Pg_op = 5e5; % Operating point pressure of actuator [Pa]

triangle_r = 0.1; % Length from center of triangle to one of the attachments [m]
m_t = 1; % Mass of triangle center [kg]
b_l =50; % Linear damping of PAMs
m_p = 0.5; % Mass of pen [kg]
y_p_com = .01; % Vertical height of COM of pen [m]
k_p = 1e4; % Radial stiffness of pen [N/m]
b_p = 10; % Linear damping of pen tip on table
l_pen = .1; %[m]

tau = 0.01; % Fluid lowpass time constant [sec]

global sys_params
sys_params.tau = tau;
sys_params.m_t = m_t;
sys_params.m_p = m_p;
sys_params.Pg_op = Pg_op;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Actuator force linearization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define force equation from Kang paper
syms Pg D_o q E a_o
F_nl=Pg*D_o^2*pi/4*(3*(1-q*E)^2/tan(a_o)^2-1/(sin(a_o)^2));

% Strain definition
syms L L_o
E=(L_o-L)/L_o;

% Correction factor from Kang paper
syms cq1 cq2
q= 1+ cq1*exp(cq2*Pg);

% Evaluate for system constants
D_o = 0.010; %meters (from datasheet)
a_o = 23*pi/180; %pi/6;  %rad (GUESS)
L_o = 0.300; %meters - measured in lab
cq1= 3; % (FIT FROM DATA)
cq2= -0.000002;  %(FIT FROM DATA)


% Equations for each of the three actuators
syms L1 L2 L3 Pg1 Pg2 Pg3 L1_dot L2_dot L3_dot
F_nl=subs(F_nl);
L= L1;
Pg=Pg1;
F_nl_1 =subs(F_nl-b_l*L1_dot);

L= L2;
Pg=Pg2;
F_nl_2 =subs(F_nl-b_l*L2_dot);

L= L3;
Pg=Pg3;
F_nl_3 =subs(F_nl-b_l*L3_dot);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kinematics of V2 actuator setup
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define fixed coordinates where actuators attach to base
% Center coordinate is (0,0)
b_1 = [0; (l_op+triangle_r); 0];
b_2 = [(l_op+triangle_r)*sin(2*pi/3); (l_op+triangle_r)*cos(2*pi/3); 0];
b_3 = [(l_op+triangle_r)*sin(-2*pi/3); (l_op+triangle_r)*cos(-2*pi/3); 0];

% Coordinate transformation between global coordinates and triangle
% coordinates 
syms theta 
theta=0;
omega=0;

t_b = [cos(theta), -sin(theta), 0;
     sin(theta), cos(theta), 0;
     0, 0, 1]; % Base to triangle

%b_t = inv(t_b); % Triangle to base
b_t = [cos(theta), sin(theta), 0;
     -sin(theta), cos(theta), 0;
     0, 0, 1];

% Solve for angles phi_1, 2, 3 given center of triangle position (base coordinates) and theta
syms x_b y_b vx_b vy_b x_p y_p vx_p vy_p

r1 = b_t*[triangle_r*sin(0); triangle_r*cos(0);0];    % Vector in base coordinates from center to attachment
f1 = b_1 - ([x_b;y_b;0]+r1);        % Vector from actuator start to end
L1 = norm(f1);                      % Length of actuator
phi1 = -atan2(f1(2),f1(1))+pi/2;    % Angle relative to global vertical
phi1_rel = phi1-theta;              % Angle relative to triangle nominal
v_pt1 = [vx_b; vy_b; 0] + cross([0;0;-omega],r1); % Velocity in base coordinates of attachment point
L1_dot = dot(f1,v_pt1)/L1;          % For rate of change only look at dot product

r2 = b_t*[triangle_r*sin(2*pi/3); triangle_r*cos(2*pi/3);0];
f2 = b_2 - ([x_b;y_b;0]+r2);
L2 = norm(f2);
phi2 = -atan2(f2(2),f2(1))+pi/2; 
phi2_rel = phi2-(theta+2*pi/3);
v_pt2 = [vx_b; vy_b; 0] + cross([0;0;-omega],r2); % Velocity in base coordinates of attachment point
L2_dot = dot(f2,v_pt2)/L2;          % For rate of change only look at dot product

r3 = b_t*[triangle_r*sin(-2*pi/3); triangle_r*cos(-2*pi/3);0];
f3 = b_3 - ([x_b;y_b;0]+r3);
L3 = norm(f3);
phi3 = -atan2(f3(2),f3(1))+pi/2; 
phi3_rel = phi3-(theta+4*pi/3);
v_pt3 = [vx_b; vy_b; 0] + cross([0;0;-omega],r3); % Velocity in base coordinates of attachment point
L3_dot = dot(f3,v_pt3)/L3;          % For rate of change only look at dot product



% Resolve for force equations given theta coordinates instead of L
F_nl_1 =subs(F_nl_1);
F_nl_2 =subs(F_nl_2);
F_nl_3 =subs(F_nl_3);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Equations of motion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Force in x,y dir
f_x = F_nl_1 * f1(1)/sqrt(f1(1)^2+f1(2)^2) + F_nl_2 * f2(1)/sqrt(f2(1)^2+f2(2)^2) + F_nl_3 * f3(1)/sqrt(f3(1)^2+f3(2)^2) + k_p * (x_p-x_b);
f_y = F_nl_1 * f1(2)/sqrt(f1(1)^2+f1(2)^2) + F_nl_2 * f2(2)/sqrt(f1(1)^2+f1(2)^2) + F_nl_3 * f3(2)/sqrt(f1(1)^2+f1(2)^2) + k_p * (y_p-y_b);


% Pen force in x,y dir
f_x_pen = k_p * (x_b-x_p) - b_p * vx_p;
f_y_pen = k_p * (y_b-y_p) - b_p * vy_p;

% Save for eqn to co
sys_params.f_x_nl = f_x; 
sys_params.f_y_nl = f_y;
sys_params.f_x_p = f_x_pen;
sys_params.f_y_p = f_y_pen;



% Take partial derivatives
dfx_dPg1= diff(f_x,Pg1);
dfx_dPg2= diff(f_x,Pg2);
dfx_dPg3= diff(f_x,Pg3);
dfx_dxb= diff(f_x,x_b);
dfx_dyb= diff(f_x,y_b);
dfx_dtheta= diff(f_x,theta);
dfx_dvxb= diff(f_x,vx_b);
dfx_dvyb= diff(f_x,vy_b);
dfx_domega= diff(f_x,omega);
dfx_dxp =  diff(f_x,x_p);
dfx_dyp =  diff(f_x,y_p);

dfy_dPg1= diff(f_y,Pg1);
dfy_dPg2= diff(f_y,Pg2);
dfy_dPg3= diff(f_y,Pg3);
dfy_dxb= diff(f_y,x_b);
dfy_dyb= diff(f_y,y_b);
dfy_dtheta= diff(f_y,theta);
dfy_dvxb= diff(f_y,vx_b);
dfy_dvyb= diff(f_y,vy_b);
dfy_domega= diff(f_y,omega);
dfy_dxp =  diff(f_y,x_p);
dfy_dyp =  diff(f_y,y_p);



dfx_pen_dxb =diff(f_x_pen,x_b);
dfx_pen_dyb =diff(f_x_pen,y_b);
dfx_pen_dxp =diff(f_x_pen,x_p);
dfx_pen_dyp =diff(f_x_pen,y_p);
dfx_pen_dvxp =diff(f_x_pen,vx_p);
dfx_pen_dvyp =diff(f_x_pen,vy_p);

dfy_pen_dxb =diff(f_y_pen,x_b);
dfy_pen_dyb =diff(f_y_pen,y_b);
dfy_pen_dxp =diff(f_y_pen,x_p);
dfy_pen_dyp =diff(f_y_pen,y_p);
dfy_pen_dvxp =diff(f_y_pen,vx_p);
dfy_pen_dvyp =diff(f_y_pen,vy_p);



% Evaluate partials at operating point
x_b = 0;
y_b = 0;
vx_b =0;
vy_b = 0;
Pg1 = Pg_op;
Pg2 = Pg_op;
Pg3 = Pg_op;
x_p = 0;
y_p = 0;
vx_p = 0;
vy_p=0;

dfx_dPg1_op= double(subs(dfx_dPg1));
dfx_dPg2_op= double(subs(dfx_dPg2));
dfx_dPg3_op= double(subs(dfx_dPg3));
dfx_dxb_op= double(subs(dfx_dxb));
dfx_dyb_op= double(subs(dfx_dyb));
dfx_dtheta_op= double(subs(dfx_dtheta));
dfx_dvxb_op = double(subs(dfx_dvxb));
dfx_dvyb_op = double(subs(dfx_dvyb));
dfx_domega_op = double(subs(dfx_domega));
dfx_dxp_op = double(subs(dfx_dxp));
dfx_dyp_op = double(subs(dfx_dyp));

dfy_dPg1_op= double(subs(dfy_dPg1));
dfy_dPg2_op= double(subs(dfy_dPg2));
dfy_dPg3_op= double(subs(dfy_dPg3));
dfy_dxb_op= double(subs(dfy_dxb));
dfy_dyb_op= double(subs(dfy_dyb));
dfy_dtheta_op= double(subs(dfy_dtheta));
dfy_dvxb_op = double(subs(dfy_dvxb));
dfy_dvyb_op = double(subs(dfy_dvyb));
dfy_domega_op = double(subs(dfy_domega));
dfy_dxp_op = double(subs(dfy_dxp));
dfy_dyp_op = double(subs(dfy_dyp));


dfx_pen_dxb_op =double(subs(dfx_pen_dxb));
dfx_pen_dyb_op =double(subs(dfx_pen_dyb));
dfx_pen_dxp_op =double(subs(dfx_pen_dxp));
dfx_pen_dyp_op =double(subs(dfx_pen_dyp));
dfx_pen_dvxp_op =double(subs(dfx_pen_dvxp));
dfx_pen_dvyp_op =double(subs(dfx_pen_dvyp));

dfy_pen_dxb_op =double(subs(dfy_pen_dxb));
dfy_pen_dyb_op =double(subs(dfy_pen_dyb));
dfy_pen_dxp_op =double(subs(dfy_pen_dxp));
dfy_pen_dyp_op =double(subs(dfy_pen_dyp));
dfy_pen_dvxp_op =double(subs(dfy_pen_dvxp));
dfy_pen_dvyp_op =double(subs(dfy_pen_dvyp));




% Taylor series expansion
syms x_b y_b vx_b vy_b Pg1 Pg2 Pg3  x_p y_p vx_p vy_p
dfx= dfx_dPg1_op * Pg1 +  dfx_dPg2_op * Pg2 + dfx_dPg3_op * Pg3 + dfx_dxb_op * x_b...
    + dfx_dyb_op * y_b + dfx_dtheta_op*theta + dfx_dvxb_op*vx_b + dfx_dvyb_op*vy_b...
    + dfx_domega_op*omega + dfx_dxp*x_p + dfx_dyp*y_p;
dfy= dfy_dPg1_op * Pg1 +  dfy_dPg2_op * Pg2 + dfy_dPg3_op * Pg3 + dfy_dxb_op * x_b...
    + dfy_dyb_op * y_b + dfy_dtheta_op*theta+ dfy_dvxb_op*vx_b + dfy_dvyb_op*vy_b...
    + dfy_domega_op*omega+ dfy_dxp*x_p + dfy_dyp*y_p;


dfx_pen = dfx_pen_dxb_op*x_b + dfx_pen_dyb_op*y_b + dfx_pen_dxp_op*x_p +dfx_pen_dyp_op*y_p...
    +dfx_pen_dvxp_op*vx_p + dfx_pen_dvyp_op*vy_p;

dfy_pen = dfy_pen_dxb_op*x_b + dfy_pen_dyb_op*y_b + dfy_pen_dxp_op*x_p +dfy_pen_dyp_op*y_p...
    +dfy_pen_dvxp_op*vx_p + dfy_pen_dvyp_op*vy_p;

sys_params.dfx=dfx;
sys_params.dfy=dfy;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% State Space Form
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% State definition
%  x_b=x(1);
%  y_b=x(2);
%  theta=x(3);
%  P1_g = x(4);
%  P2_g = x(5);
%  P3_g = x(6);
%  vx_b=x(7);
%  vy_b=x(8);
%  theta_dot=x(9);
%  P1_g_dot = x(10);
%  P2_g_dot = x(11);
%  P3_g_dot = x(12);



M= [m_t, 0, 0, 0;
    0, m_t, 0, 0;
    0, 0, m_p, 0;
    0, 0, 0, m_p];
K =  [dfx_dxb_op, dfx_dyb_op,  dfx_dxp_op, dfx_dyp_op, dfx_dPg1_op, dfx_dPg2_op, dfx_dPg3_op, dfx_dvxb_op, dfx_dvyb_op, 0, 0;
    dfy_dxb_op, dfy_dyb_op, dfy_dxp_op, dfy_dyp_op, dfy_dPg1_op, dfy_dPg2_op, dfy_dPg3_op, dfy_dvxb_op, dfy_dvyb_op, 0, 0;
    dfx_pen_dxb_op, dfx_pen_dyb_op, dfx_pen_dxp_op, dfx_pen_dyp_op, 0, 0, 0, 0, 0, dfx_pen_dvxp_op, dfx_pen_dvyp_op;
    dfy_pen_dxb_op, dfy_pen_dyb_op, dfy_pen_dxp_op, dfy_pen_dyp_op, 0, 0, 0, 0, 0, dfy_pen_dvxp_op, dfy_pen_dvyp_op];
% B = [0 0 0 0 0 0 0 -b 0 0 0;
%      0 0 0 0 0 0 0 0 -b 0 0;
%      0 0 0 0 0 0 0 0 0 0 0;
%      0 0 0 0 0 0 0 0 0 0 0];


A=[zeros(4,7), eye(4);
    zeros(3,4),-1/tau*eye(3), zeros(3,4);
    inv(M)*(K)];


B=[zeros(4,3);
    [1/tau, 0, 0;
    0, 1/tau, 0;
    0, 0, 1/tau];
    zeros(4,3)];


C = [1 0 0 0 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0 0 0 0;
    0 0 0 0 1 1 1 0 0 0 0];
%     l_pen/y_p_com 0 -l_pen/y_p_com 0 0 0 0 0 0 0 0;
%     0 l_pen/y_p_com 0 -l_pen/y_p_com 0 0 0 0 0 0 0;
    
P = [l_pen/y_p_com 0 -l_pen/y_p_com 0 0 0 0 0 0 0 0;
     0 l_pen/y_p_com 0 -l_pen/y_p_com 0 0 0 0 0 0 0];



D = zeros(3,3);


sys = ss(A,B,C,D);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% System analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Look at system controllability
[ABAR,BBAR,CBAR,T,ctrb_states]=ctrbf(A,B,C);
fprintf('Number of controllable states: %0.0f\n',sum(ctrb_states));
[ABAR,BBAR,CBAR,T,obsv_states]=obsvf(A,B,C);
fprintf('Number of observable states: %0.0f\n',sum(obsv_states));


% Look at system in modal form
csys = canon(sys,'modal');
sys.InputName = {'Pin_1','Pin_2','Pin_3'};
sys.OutputName = {'x_b','y_b','Pg_1'};

% Mode shapes (no damping)
A_nd=[A(1:7,:);
    [A(8:11,1:7),zeros(4)]];
[T,V]=eig(A_nd);
mode_freq = diag(V);

% bode plot
figure(1);
bode(sys);
title('Bode plot for MIMO system')

% Pz plot
figure()
iopzmap(sys);

% Step response for each input
opt = stepDataOptions('InputOffset',0,'StepAmplitude',1e5);
[y,t,x]=step(sys,3);

% Response for step in Pin1
figure();

subplot(4,1,1); plot(t,x(:,1,1));
xlabel('Time [s]');
ylabel('x_b [m]');
subplot(4,1,2); plot(t,x(:,2,1));
xlabel('Time [s]');
ylabel('y_b [m]');
subplot(4,1,3); plot(t,x(:,3,1));
xlabel('Time [s]');
ylabel('x_p [m]');
subplot(4,1,4); plot(t,x(:,4,1));
xlabel('Time [s]');
ylabel('y_p [m]');




% subplot(3,2,2); plot(t,x(:,4,1));
% xlabel('Time [s]');
% ylabel('P1 [Pa]');
% subplot(3,2,4); plot(t,x(:,5,1));
% xlabel('Time [s]');
% ylabel('P2 [Pa]');
% subplot(3,2,6); plot(t,x(:,6,1));
% xlabel('Time [s]');
% ylabel('P3 [Pa]');
% sgtitle('Step response for step in input pressure P1')

% Initial condition response
x0=[.15; .15;.15; .15; Pg_op; Pg_op; Pg_op; 0; 0; 0; 0];
[y,t,x]=initial(sys,xabs2lin(x0),0.5);
x=xlin2abs(x);

figure();
subplot(4,1,1); plot(t,x(:,1,1));
title('IC Response (linear)')
xlabel('Time [s]');
ylabel('x_b [m]');
subplot(4,1,2); plot(t,x(:,2,1));
xlabel('Time [s]');
ylabel('y_b [m]');
subplot(4,1,3); plot(t,x(:,3,1));
xlabel('Time [s]');
ylabel('x_p [m]');
subplot(4,1,4); plot(t,x(:,4,1));
xlabel('Time [s]');
ylabel('y_p [m]');


%v=draw_sys('IC_response',l,r,t,x(:,1,1),x(:,2,1),x(:,3,1),x(:,4,1));

t_sim=linspace(0,0.5,1000);

sys_params.t = t_sim;
sys_params.K=0;
sys_params.u = @(x,K,r) ulin2abs(-K*xabs2lin(x));
sys_params.r = @(t) t.*0;


[t_nlin, x_nlin] = ode45(@(t,x) sys_nl(t,x), t_sim, x0);


figure();
subplot(4,1,1); plot(t,x(:,1,1));
hold on;
plot(t_nlin,x_nlin(:,1));
legend('Linear', 'Nonlinear')
title('IC Response')
xlabel('Time [s]');
ylabel('x_b [m]');
subplot(4,1,2); plot(t,x(:,2));
hold on;
plot(t_nlin,x_nlin(:,2,1));
legend('Linear', 'Nonlinear')
xlabel('Time [s]');
ylabel('y_b [m]');
subplot(4,1,3); plot(t,x(:,3));
hold on;
plot(t_nlin,x_nlin(:,3,1));
legend('Linear', 'Nonlinear')
xlabel('Time [s]');
ylabel('x_p [m]');
subplot(4,1,4); plot(t,x(:,4));
hold on;
plot(t_nlin,x_nlin(:,4,1));
legend('Linear', 'Nonlinear')
xlabel('Time [s]');
ylabel('y_p [m]');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% LQR
rho = 1/1; % weighting variable

% State definition
%  x_b=x(1);
%  y_b=x(2);
%  x_p=x(3);
%  y_p = x(4)
%  P1_g = x(5);
%  P2_g = x(6);
%  P3_g = x(7);
%  vx_b=x(8);
%  vy_b=x(9);
%  vx_p=x(10);
%  vy_p=x(11);

% State errors
x_b_max = 1e-3;
y_b_max = 1e-3;
x_p_max = 0.1e-3;
y_p_max = 0.1e-3;
P1_g_max =1000;
P2_g_max =1000;
P3_g_max =1000;
vx_b_max = 1e-3;
vy_b_max = 1e-3;
vx_p_max = 0.1e-3;
vy_p_max = 0.1e-3;



Qy=1./[x_b_max, y_b_max, x_p_max, y_p_max, P1_g_max, P2_g_max, P3_g_max, vx_b_max, vy_b_max, vx_p_max, vy_p_max].^2;
Q =diag(Qy);


Pin_max = .001;
Ry = rho*1./[Pin_max, Pin_max, Pin_max].^2;

R = diag(Ry);

[K,S,CLP] = lqr(sys,Q,R);



rad = .02;
t_sim=linspace(0,2*pi/10,200);
w=10;
y_ss = [rad*cos(w*t_sim);rad*sin(w*t_sim);zeros(1,length(t_sim))];
r = -inv(C*inv(A-B*K)*B)*y_ss;
A_cl = A-B*K;

sys_params.K = K;
sys_params.u = @(x,K,r) ulin2abs(r-K*xabs2lin(x));
sys_params.r = @(t) pchip(t_sim,r,t);

sys_cl = ss(A_cl,B,C,D);

figure()
iopzplot(sys_cl)

[y,t,x]=lsim(sys_cl,r,t_sim);
x=xlin2abs(x);

x0=[y_ss(1,1); y_ss(2,1); y_ss(1,1); y_ss(2,1); Pg_op; Pg_op; Pg_op; 0; 0; 0; 0];
x0=xfill_init_pressures(x0)
[t_nlin, x_nlin] = ode45(@(t,x) sys_nl(t,x), t_sim, x0);

figure();
subplot(4,1,1); plot(t,x(:,1,1));
hold on;
plot(t_nlin,x_nlin(:,1));
legend('Linear', 'Nonlinear')
title('Circle trajectory')
xlabel('Time [s]');
ylabel('x_b [m]');
subplot(4,1,2); plot(t,x(:,2));
hold on;
plot(t_nlin,x_nlin(:,2,1));
legend('Linear', 'Nonlinear')
xlabel('Time [s]');
ylabel('y_b [m]');
subplot(4,1,3); plot(t,x(:,3));
hold on;
plot(t_nlin,x_nlin(:,3,1));
legend('Linear', 'Nonlinear')
xlabel('Time [s]');
ylabel('x_p [m]');
subplot(4,1,4); plot(t,x(:,4));
hold on;
plot(t_nlin,x_nlin(:,4,1));
legend('Linear', 'Nonlinear')
xlabel('Time [s]');
ylabel('y_p [m]');


figure();
subplot(3,1,1); plot(t,x(:,5,1));
xlabel('Time [s]');
ylabel('P1 [Pa]');
subplot(3,1,2); plot(t,x(:,6,1));
xlabel('Time [s]');
ylabel('P2 [Pa]');
subplot(3,1,3); plot(t,x(:,7,1));
xlabel('Time [s]');
ylabel('P3 [Pa]');

draw_sys('traj_response_nl',l_op,triangle_r,t_nlin,x_nlin(:,1,1),x_nlin(:,2,1),x_nlin(:,3,1),x_nlin(:,4,1));
plot(rad*cos(w*t_sim),rad*sin(w*t_sim))

%%% Adding observer
cl_poles = eig(A_cl);
L = place(A',C',3*cl_poles);


A_w_obsv = [A -B*K;L'*C (A-B*K-L'*C)];
B_w_obsv = [B;B];
C_w_obsv = [C zeros(size(C))];
sys_w_obsv = ss(A_w_obsv, B_w_obsv, C_w_obsv, D);




% 
% 
% figure();
% step(sys_cl,3,opt)
% title('Step response after LQR')
% 
% figure();
% pzmap(sys_cl)
% title("LQR Closed Loop Poles")
% 
% % Initial condition response
% x0=[.01; .01; .05; 0; 0; 0; 0; 0; 0];
% [y,t,x]=initial(sys_cl,x0,5);
% 
% figure();
% 
% 
% subplot(3,2,1); plot(t,x(:,1,1));
% xlabel('Time [s]');
% ylabel('x_b [m]');
% subplot(3,2,3); plot(t,x(:,2,1));
% xlabel('Time [s]');
% ylabel('y_b [m]');
% subplot(3,2,5); plot(t,x(:,3,1));
% xlabel('Time [s]');
% ylabel('theta [rad]');
% 
% 
% subplot(3,2,2); plot(t,x(:,4,1));
% xlabel('Time [s]');
% ylabel('P1 [Pa]');
% subplot(3,2,4); plot(t,x(:,5,1));
% xlabel('Time [s]');
% ylabel('P2 [Pa]');
% subplot(3,2,6); plot(t,x(:,6,1));
% xlabel('Time [s]');
% ylabel('P3 [Pa]');
% sgtitle('Initial condition response (LQR)')

function xdot = sys_nl(t,x)
    global sys_params



    % State extraction
    x_b=x(1);
    y_b=x(2);
    x_p=x(3);
    y_p = x(4);
    Pg1 = x(5);
    Pg2 = x(6);
    Pg3 = x(7);
    vx_b=x(8);
    vy_b=x(9);
    vx_p=x(10);
    vy_p=x(11);


    % Extract important sys parameters
    tau = sys_params.tau;
    m_t = sys_params.m_t;
    m_p = sys_params.m_p;



    % Force calculation
    f_x=double(subs(sys_params.f_x_nl));
    f_y=double(subs(sys_params.f_y_nl));
    f_x_p = double(subs(sys_params.f_x_p));
    f_y_p = double(subs(sys_params.f_y_p));

    % State equations
    r = sys_params.r(t);
    u = sys_params.u(x,sys_params.K,r);


    % Return the state derivatives
    xdot = [vx_b;
            vy_b;
            vx_p;
            vy_p;
            -1/tau * (Pg1 - u(1));
            -1/tau * (Pg2 - u(2));
            -1/tau * (Pg3 - u(3));
            f_x/m_t;
            f_y/m_t;
            f_x_p/m_p;
            f_y_p/m_p];
end

function x_lin = xabs2lin(x_abs)
    global sys_params
    
    x_lin = x_abs;
    x_lin(5) = x_lin(5)-sys_params.Pg_op;
    x_lin(6) = x_lin(6)-sys_params.Pg_op;
    x_lin(7) = x_lin(7)-sys_params.Pg_op;

end

function x_abs = xlin2abs(x_lin)
    global sys_params
    
    x_abs = x_lin;
    x_abs(5) = x_lin(5)+sys_params.Pg_op;
    x_abs(6) = x_lin(6)+sys_params.Pg_op;
    x_abs(7) = x_lin(7)+sys_params.Pg_op;

end

function u_lin = uabs2lin(u_abs)
    global sys_params
    
    u_lin = u_abs-sys_params.Pg_op;
    

end


function u_abs = ulin2abs(u_lin)
    global sys_params
    
    u_abs = u_lin+sys_params.Pg_op;
    
end


function x_with_p = xfill_init_pressures(x)
    global sys_params

    f_x_nl = sys_params.f_x_nl;
    f_y_nl = sys_params.f_y_nl;

    % State extraction
    x_b=x(1);
    y_b=x(2);
    x_p=x(3);
    y_p =x(4);
    vx_b=x(8);
    vy_b=x(9);
    vx_p=x(10);
    vy_p=x(11);

    syms Pg1 Pg2 Pg3
    eqn3= Pg1 + Pg2 + Pg3 ==3*sys_params.Pg_op;
    soln = vpasolve([subs(f_x_nl)==0, subs(f_y_nl)==0, eqn3],[Pg1, Pg2, Pg3]);

    x_with_p = x;
    x_with_p(5)=soln.Pg1(1)
    x_with_p(6)=soln.Pg2(1)
    x_with_p(7)=soln.Pg3(1)

end
