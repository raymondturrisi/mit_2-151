function lin_model = get_lin_model(p_0, L_0)
    %linearization points
    L_a = 0.3; %length of actuator
    P_g0 = p_0; 
    
    %Correction Factor - Kang
    C_q1 = 3; %unitless, referencing kang's paper for ballpark region estimate
    C_q2 = -0.000002; %unitless
    q = @(P) 1 + C_q1 * exp(C_q2*P); %q correction factor
    eps = @(L) (L_a - L)/(L_a); %strain
    alpha_0 = 23*pi/180; %braid angle, degrees to radians
    D_0 = 0.010; %meters
    C_1 = (pi*D_0^2)/(4.0);
    C_2 = (3)/(tan(alpha_0)^2);
    C_3 = (1)/(sin(alpha_0)^2);
    C_4 = C_1*C_2;
    C_5 = C_1*C_3;
    
    Q1p1 = C_4-C_5-2*C_4*((L_a-L_0)/L_a)-2*C_4*((L_a-L_0)/(L_a))*C_q1*(exp(C_q2*P_g0)+(P_g0)*C_q2*exp(C_q2*P_g0));
    Q1p2 = C_4+C_4*(C_q1^2)*(exp(2*C_q2*P_g0)+2*(P_g0)*C_q2*exp(2*C_q2*P_g0));
    Q1p3 = 2*C_4*C_q1*(exp(C_q2*P_g0)+(P_g0)*C_q2*exp(C_q2*P_g0));
    Q1pend = ((L_a^2 + L_0^2 - 2*L_0*L_a)/(L_a^2));
    Q_1=Q1p1+(Q1p2 + Q1p3)*Q1pend;
    Q_2 = (2*P_g0*C_4)/(L_a) + ((2*P_g0*C_4*C_q1)/(L_a))*exp(C_q2*P_g0)+(P_g0*C_4+P_g0*C_4*(C_q1^2)*exp(2*C_q2*P_g0) + 2*P_g0*C_4*C_q1*exp(C_q2*P_g0))*((2*L_0)/(L_a^2) - 2/L_a);

    lin_model = @(dP, dL) Q_1*(dP)+Q_2*(dL);
end

