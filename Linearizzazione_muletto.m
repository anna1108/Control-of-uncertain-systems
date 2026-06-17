% Variabili simboliche
syms y_sym theta_sym phi_sym psi_sym phi_dot_sym psi_dot_sym vx_sym rp_sym L_sym a_sym b_sym c_phi_sym c_psi_sym Ipz_sym Ipy_sym tau_phi_sym tau_psi_sym phi_dot_dot_sym psi_dot_dot_sym

% Definizione stato e ingressi
z = [y_sym theta_sym phi_dot_sym psi_sym psi_dot_sym];
u = [tau_phi_sym tau_psi_sym];

% Equilibrio
zeq = [0 0 vx_sym/rp_sym 0 0];
ueq = [c_phi_sym*phi_dot_sym 0];

% z_dot
z_dot = [rp_sym*phi_dot_sym*sin(theta_sym)*cos(psi_sym);
         -(rp_sym/L_sym)*phi_dot_sym*sin(psi_sym);
         (tau_phi_sym + Ipz_sym*(rp_sym/L_sym)*psi_dot_dot_sym*sin(psi_sym)-(b_sym*(rp_sym/L_sym)^2-a_sym*rp_sym^2)*phi_dot_sym*psi_dot_sym*sin(psi_sym)*cos(psi_sym)-c_phi_sym*phi_dot_sym)/(a_sym*rp_sym^2*(cos(psi_sym))^2+b_sym*(rp_sym/L_sym)^2*(sin(psi_sym))^2+Ipy_sym);
         psi_dot_sym;
         (tau_psi_sym+Ipz_sym*(rp_sym/L_sym)*phi_dot_dot_sym*sin(psi_sym)+Ipz_sym*(rp_sym/L_sym)*psi_dot_sym*phi_dot_sym*cos(psi_sym)-c_psi_sym*psi_dot_sym)/Ipz_sym];

% Derivate
Alin = jacobian(z_dot,z);
Blin = jacobian(z_dot,u);

% Sostituisco equilibrio

Aeq = subs(Alin,z,zeq);
Aeq = subs(Aeq,[phi_dot_dot_sym,psi_dot_dot_sym],[0 0]);
Beq = subs(Blin,z,zeq);

% % Matrici con valori dei parametri
% 
% Areal = double(subs(Aeq, [rp_sym, a_sym, b_sym, Ipz_sym, Ipy_sym, c_phi_sym, c_psi_sym, L_sym, vx_sym, phi_dot_dot_sym, psi_dot_dot_sym], ...
%                        [0.3, 19.1667, 2.8575, 0.23, 0.15, 0.01, 0.02, 1, 10, 0, 0]));
% 
% Breal = double(subs(Beq, [rp_sym, a_sym, b_sym, Ipz_sym, Ipy_sym, c_phi_sym, c_psi_sym, L_sym, vx_sym, phi_dot_dot_sym, psi_dot_dot_sym], ...
%                        [0.3, 19.1667, 2.8575, 0.23, 0.15, 0.01, 0.02, 1, 10, 0, 0]));
% 
