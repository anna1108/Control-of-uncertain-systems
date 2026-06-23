clc
clear all
close all

Parameter_CSI;

%% Mix-sensitivity Hinf controller design (hinfsyn)
s = tf('s');

%% Nominal model

G = G_tot_nom;

%% Definition of weight function parameters
A_hinfsyn  = 1e-4;
M_hinfsyn  = 1.2;
wB_hinfsyn_phi = 1;
wB_hinfsyn_psi = 1;

% Performance weight matrix for S
wP_hinfsyn_phi = 1/1 * (s/M_hinfsyn + wB_hinfsyn_phi)/(s + wB_hinfsyn_phi*A_hinfsyn);
wP_hinfsyn_psi = 1/1 * (s/M_hinfsyn + wB_hinfsyn_psi)/(s + wB_hinfsyn_psi*A_hinfsyn);
WP_hinfsyn = blkdiag(wP_hinfsyn_phi, wP_hinfsyn_psi); % Diagonal weight matrix for S

%% Generalized plant P by using 

G.u = 'u';
G.y = 'yG';

WP_hinfsyn.u = 'e';
WP_hinfsyn.y = 'z1';

Sum1 = sumblk('y=w+yG',2);
Sum3 = sumblk('e= -y',2);

P_hinfsyn = connect(G, WP_hinfsyn, Sum1,Sum3,...
    {'w','u'},{'z1','e'});

[K_hinfsyn, CL_hinfsyn, gamma_hinfsyn, info] = hinfsyn(P_hinfsyn,2,2);

gamma_hinfsyn;   % Display peak value

K_hinfsyn        = (K_hinfsyn);
K_mix = K_hinfsyn;
K_hinfsyn_tf = tf(K_hinfsyn);

RL_nom_K_hinfsyn = (sys_tot_nom*K_hinfsyn);
sys_c_K_hinfsyn  = (feedback(RL_nom_K_hinfsyn, eye(p)));
G_c_K_hinfsyn    = (tf(sys_c_K_hinfsyn));

% Compute S
S_hinfsyn_struct = loopsens(sys_tot_nom, K_hinfsyn);
S_hinfsyn_o = (S_hinfsyn_struct.So);
T_hinfsyn_o = (S_hinfsyn_struct.To);

poles_sys_cl_K_hinfsyn = pole(sys_c_K_hinfsyn);
zeros_sys_cl_K_hinfsyn = tzero(sys_c_K_hinfsyn);

%-----PLOT 1-----
% Closed-loop response plot
figure  % System output plot
step(sys_c_K_hinfsyn*[1 0]','b-')
xlim([0 30]);
grid on
title('Step response - Input 1');

%-----PLOT 2-----
% Closed-loop response plot
figure  % System output plot
step(sys_c_K_hinfsyn*[0 1]','b-')
xlim([0 30]);
grid on
title('Step response - Input 2');

% Plot singular values to verify that specifications are met
%-----PLOT 3-----
figure % Singular values of S vs inverse of WP
sigmaplot(S_hinfsyn_o,'b',inv(WP_hinfsyn),'r--');
grid on
legend('\sigma(S)', '\sigma(WP^{-1})', 'Interpreter', 'latex');
title('Sensitivity vs target');