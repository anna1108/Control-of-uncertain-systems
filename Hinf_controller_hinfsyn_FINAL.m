clc
clear all
close all

Parameter_CSI;

%% Progetto del controllore mix-sensitivity Hinf
s = tf('s');

%% Nominal model

G = G_tot_nom;

%% Definizione dei parametri per le funzioni peso
A_hinfsyn  = 1e-4;
M_hinfsyn  = 1.2;
wB_hinfsyn_phi = 1;
wB_hinfsyn_psi = 1;

% Matrice di peso per le prestazioni, sulla S
wP_hinfsyn_phi = 1/1 * (s/M_hinfsyn + wB_hinfsyn_phi)/(s + wB_hinfsyn_phi*A_hinfsyn);
wP_hinfsyn_psi = 1/1 * (s/M_hinfsyn + wB_hinfsyn_psi)/(s + wB_hinfsyn_psi*A_hinfsyn);
WP_hinfsyn = blkdiag(wP_hinfsyn_phi, wP_hinfsyn_psi); % Matrice diagonale di peso per S

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

gamma_hinfsyn;   % Visualizzo il valore del picco 

K_hinfsyn        = (K_hinfsyn);
K_hinfsyn_tf = tf(K_hinfsyn);

RL_nom_K_hinfsyn = (sys_tot_nom*K_hinfsyn);
sys_c_K_hinfsyn  = (feedback(RL_nom_K_hinfsyn, eye(p)));
G_c_K_hinfsyn    = (tf(sys_c_K_hinfsyn));

% Calcolo di S
S_hinfsyn_struct = loopsens(sys_tot_nom, K_hinfsyn);
S_hinfsyn_o = (S_hinfsyn_struct.So);
T_hinfsyn_o = (S_hinfsyn_struct.To);

poli_sys_cl_K_hinfsyn = pole(sys_c_K_hinfsyn);
zeri_sys_cl_K_hinfsyn = tzero(sys_c_K_hinfsyn); 

%-----GRAFICO 1-----
% Plot risposta a ciclo chiuso
figure  % Plot di uscita del sistema
step(sys_c_K_hinfsyn*[1 0]','b-')
xlim([0 30]);
grid on
title('Risposta al gradino - Ingresso 1');

%-----GRAFICO 2-----
% Plot risposta a ciclo chiuso
figure  % Plot di uscita del sistema
step(sys_c_K_hinfsyn*[0 1]','b-')
xlim([0 30]);
grid on
title('Risposta al gradino - Ingresso 2');

%Grafico i valori singolari vari per vedere se le specifiche sono
%rispettate
%-----GRAFICO 3-----
figure % Plot dei valori singolari di S a confronto con l'inversa della WP
sigmaplot(S_hinfsyn_o,'b',inv(WP_hinfsyn),'r--');
grid on
legend('\sigma(S)', '\sigma(WP^{-1})', 'Interpreter', 'latex');
title('Confronto fra Sensitività e obbiettivo');