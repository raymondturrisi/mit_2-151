function dx = nlim_sim(t,x,sys_consts)
%Get the next measurement from the plant


%{
    Non linear simulation
    - Given a pressure, solve for what the length will be if the force was
    equal to the mass of the system
    - With this length, derive a stiffness constant with the equilibrium
    length 
        - (L_0+dL): Current length
        - L_f: Equilibrium length
        - F = ()

%}


Pg = x(1);
dL = x(2);
L_dot = x(3);

P_gs = sys_consts.p_eq+Pg;
g = 9.81;
q = sys_consts.q;
eps = sys_consts.eps;
C_1 = sys_consts.C_1;
C_2 = sys_consts.C_2;
C_3 = sys_consts.C_3;
C_4 = sys_consts.C_4;
C_5 = sys_consts.C_5;
C_q1 = sys_consts.C_q1;
C_q2 = sys_consts.C_q2;

syms L_s
nlin_model_sim = [(P_gs.*C_4.*(1+(q(P_gs).^2).*(eps(L_s).^2) - 2.*q(P_gs).*eps(L_s)) - P_gs.*C_5) == g*sys_consts.m];

%Calculate free length of system when suspending just the mass at a
%pressure
L_f = double(max(solve(nlin_model_sim)));

%Find what the force applied can be at a length and a pressure
f_0 = sys_consts.nlin_model(Pg, L_f);
disp(f_0)
%Compute the stiffness from deviating about this equlibrium point
k = f_0/L_f;

%For convention we are working about a linearized point, so we have the
%free length, minus the linearized point plus the deviation from this
%point, this renders the difference betweent the free length and the
%total simulated length. 
dL_nl = (L_f-(sys_consts.l_0+dL));
f_k = k*dL_nl;
tau = sys_consts.tau;
b = sys_consts.b;
m = sys_consts.m;
u = sys_consts.u(t);
r = sys_consts.r(u);

Pg_dot = -Pg/tau + r/tau;
strain = 1-(sys_consts.L_a + dL)/sys_consts.L_a;

L_dotdot = (L_dot*b/m+g-f_k/m);

sys_consts.steps = sys_consts.steps+1;

dx = ... 
    [
    Pg_dot;
    L_dot;
    L_dotdot
    ];

end

