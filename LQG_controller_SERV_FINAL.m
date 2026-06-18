clc 
clear 
close all

Parameter_CSI;

%% LQG as 1-DOF servo-controller
% Define cost functional matrices
Q = 0.01 * eye(n);     % State weight matrix
R = 0.1 * eye(m);      % Input weight matrix

Q_QR = 10*blkdiag(Q, R);

Bnoise = eye(n);                % process noise model (Gd)
W = 0.1 * eye(size(Bnoise,2));   % process disturbance covariance matrix
V = 0.1 * eye(p);               % measurement noise covariance matrix
Q_WV = blkdiag(W, V);

QI = 0.01 * eye(p);

% compute the LQG servo-controller (1 DOF) using the 'lqg' command
K_lqg_sc = lqg(sys_tot_nom, Q_QR, Q_WV, QI, '1dof');
K_lqg_sc = minreal(K_lqg_sc);
Klqg_tf = minreal(tf(K_lqg_sc));

%% Simulation
% Open-loop TF: product of nominal AGV+actuators system G and the LQG controller
RL_nom_lqg_sc = (ss(G_tot_nom * K_lqg_sc));
% Closed-loop transfer function
sys_c_nom_lqg_sc = (feedback(RL_nom_lqg_sc, eye(p)));

%% Plots
%-----PLOT 1-----
% closed-loop step response
figure
step(sys_c_nom_lqg_sc * [1 0]')
grid on

% Compute sensitivity and complementary sensitivity
% G_tot_nom * K_lqg
S_lqg_struct_sc = loopsens(sys_tot_nom, K_lqg_sc);
S_lqg_o_sc = (S_lqg_struct_sc.So);
T_lqg_o_sc = (S_lqg_struct_sc.To);
% Compute S_lqg peak
picco_S_lqg_o_sc = hinfnorm(S_lqg_o_sc);
picco_T_lqg_o_sc = hinfnorm(T_lqg_o_sc);

%-----PLOT 2-----
G_u_lqg_sc = (K_lqg_sc * S_lqg_o_sc);

% Control effort plot
figure
step(G_u_lqg_sc * [1 0]')
grid on

%% Perturbed system plot
RL_pert_K_lqg_sc = ss(G_tot * K_lqg_sc);
sys_cp_K_lqg_sc = (feedback(RL_pert_K_lqg_sc,eye(p)));

% Compute S
S_p_lqg_struct_sc = loopsens(sys,K_lqg_sc);
S_lqgp_o_sc = (S_p_lqg_struct_sc.So);
T_lqgp_o_sc =(S_p_lqg_struct_sc.To);

%--- PLOT 3 -----
% closed-loop response
figure
step(sys_cp_K_lqg_sc * [1 1]')
grid on


%% LQG as 2-DOF servo-controller

% % compute the LQG servo-controller (2 DOF) using the 'lqg' command
K_lqg_sc2 = lqg(sys_tot_nom, Q_QR, Q_WV, QI); 
%K_lqg_sc2 = (K_lqg_sc2);

%% Simulation
% Open-loop TF: product of nominal AGV+actuators system G and the LQG controller
RL_nom_lqg_sc2 = (ss(G_tot_nom * K_lqg_sc2));
% Closed-loop transfer function
sys_c_nom_lqg_sc2 = (feedback(RL_nom_lqg_sc2, eye(p), [3 4], [1 2], +1));

%% Plots
%-----PLOT 4-----
% closed-loop step response
figure
step(sys_c_nom_lqg_sc2 * [1 1 0 0]')
grid on