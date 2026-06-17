clc 
clear 
close all

Parameter_CSI;

%% LQG in versione "servo-controller" a 1 g.d.l.
% Definisco le matrici presenti all'interno del funzionale di costo
Q = 0.01 * eye(n);     % Matrice peso sugli stati
R = 0.1 * eye(m);      % Matrice peso sugli ingressi

Q_QR = 10*blkdiag(Q, R);

Bnoise = eye(n);                % modello del rumore di processo (Gd)
W = 0.1 * eye(size(Bnoise,2));   % matrice di covarianza del disturbo di processo
V = 0.1 * eye(p);               % matrice di covarianza del rumore di misura 
Q_WV = blkdiag(W, V);

QI = 0.01 * eye(p);

% trovo il controllore LQG in versione "servo controller" a 1 g.d.l. sfruttando il comando 'lqg'
K_lqg_sc = lqg(sys_tot_nom, Q_QR, Q_WV, QI, '1dof');
K_lqg_sc = minreal(K_lqg_sc);
Klqg_tf = minreal(tf(K_lqg_sc));

%% Simulazione
% Definisco la FdT a ciclo aperto data dal prodotto tra la G nominale del
% sistema AGV+attuatori e la funzione del controllore LQG appena ottenuta
RL_nom_lqg_sc = (ss(G_tot_nom * K_lqg_sc));
%Definisco la FdT a ciclo chiuso
sys_c_nom_lqg_sc = (feedback(RL_nom_lqg_sc, eye(p)));

%% Grafici
%-----GRAFICO 1-----
% grafico della FdT a ciclo chiuso quando l'ingresso è un gradino
figure
step(sys_c_nom_lqg_sc * [1 0]')
grid on

% Trovo la sensitività e la sensitività complementare del sistema
% G_tot_nom * K_lqg
S_lqg_struct_sc = loopsens(sys_tot_nom, K_lqg_sc);
S_lqg_o_sc = (S_lqg_struct_sc.So);
T_lqg_o_sc = (S_lqg_struct_sc.To);
% Calcolo picco di S_lqg 
picco_S_lqg_o_sc = hinfnorm(S_lqg_o_sc);
picco_T_lqg_o_sc = hinfnorm(T_lqg_o_sc);

%-----GRAFICO 2-----
G_u_lqg_sc = (K_lqg_sc * S_lqg_o_sc);

% Plot dello sforzo di controllo
figure
step(G_u_lqg_sc * [1 0]')
grid on

%% Plot sistema perturbato 
RL_pert_K_lqg_sc = ss(G_tot * K_lqg_sc);
sys_cp_K_lqg_sc = (feedback(RL_pert_K_lqg_sc,eye(p)));

%Calcolo di S
S_p_lqg_struct_sc = loopsens(sys,K_lqg_sc);
S_lqgp_o_sc = (S_p_lqg_struct_sc.So);
T_lqgp_o_sc =(S_p_lqg_struct_sc.To);

%---GRAFICO 3-----
%plot risposta a ciclo chiuso
figure
step(sys_cp_K_lqg_sc * [1 1]')
grid on


%% LQG in versione "servo-controller" a 2 g.d.l.

% % trovo il controllore LQG in versione "servo controller" a 2 g.d.l. sfruttando il comando 'lqg'
K_lqg_sc2 = lqg(sys_tot_nom, Q_QR, Q_WV, QI); 
%K_lqg_sc2 = (K_lqg_sc2);

%% Simulazione
% Definisco la FdT a ciclo aperto data dal prodotto tra la G nominale del
% sistema AGV+attuatori e la funzione del controllore LQG appena ottenuta
RL_nom_lqg_sc2 = (ss(G_tot_nom * K_lqg_sc2));
%Definisco la FdT a ciclo chiuso
sys_c_nom_lqg_sc2 = (feedback(RL_nom_lqg_sc2, eye(p), [3 4], [1 2], +1));

%% Grafici
%-----GRAFICO 4-----
% grafico della FdT a ciclo chiuso quando l'ingresso è un gradino
figure
step(sys_c_nom_lqg_sc2 * [1 1 0 0]')
grid on