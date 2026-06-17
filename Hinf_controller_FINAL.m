clc
clear 
close all

Parameter_CSI;

%% Progetto del controllore mix-sensitivity Hinf
s = tf('s');

% Definizione dei parametri per le funzioni peso
A_mix  = 1e-4;
M_mix  = 1.5;
wB_mix_phi = 1;
wB_mix_psi = 1;

% Matrice di peso per le prestazioni, sulla S
wP_mix_phi = 1/1 * (s/M_mix + wB_mix_phi)/(s + wB_mix_phi*A_mix);
wP_mix_psi = 1/1 * (s/M_mix + wB_mix_psi)/(s + wB_mix_psi*A_mix);
WP_mix = blkdiag(wP_mix_phi, wP_mix_psi); % Matrice diagonale di peso per S

[K_mix, CL_mix, gamma_mix, info] = mixsyn(G_tot_nom, WP_mix);

gamma_mix;   % Visualizzo il valore del picco 

K_mix        = (K_mix);
K_mix_tf = tf(K_mix);

RL_nom_K_mix = (sys_tot_nom*K_mix);
sys_c_K_mix  = (feedback(RL_nom_K_mix, eye(p)));
G_c_K_mix    = (tf(sys_c_K_mix));

% Calcolo di S
S_mix_struct = loopsens(sys_tot_nom, K_mix);
S_mix_o = (S_mix_struct.So);
T_mix_o = (S_mix_struct.To);

poli_sys_cl_K_mix = pole(sys_c_K_mix);
zeri_sys_cl_K_mix = tzero(sys_c_K_mix); 

%-----GRAFICO 1-----
% Plot risposta a ciclo chiuso
figure  % Plot di uscita del sistema
step(sys_c_K_mix*[1 0]','b-')
xlim([0 30]);
grid on
title('Risposta al gradino - Ingresso 1');

%-----GRAFICO 2-----
% Plot risposta a ciclo chiuso
figure  % Plot di uscita del sistema
step(sys_c_K_mix*[0 1]','b-')
xlim([0 30]);
grid on
title('Risposta al gradino - Ingresso 2');

%Grafico i valori singolari vari per vedere se le specifiche sono
%rispettate
%-----GRAFICO 3-----
figure % Plot dei valori singolari di S a confronto con l'inversa della WP
sigmaplot(S_mix_o,'b',inv(WP_mix),'r--');
grid on
legend('\sigma(S)', '\sigma(WP^{-1})', 'Interpreter', 'latex');
title('Confronto fra Sensitività e obbiettivo');