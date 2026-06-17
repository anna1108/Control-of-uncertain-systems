clc
close all
clear all

Parameter_CSI;

%% Mu-analysis
%% Modello nominale
G = tf(sys*G_act);
G = G_tot;

s = tf('s');

%% Pesi sull'incertezza
WI = WI_tot;

%% Pesi sulle performance

% Definizione dei parametri per le funzioni peso
A_dk  = 1e-4;
M_dk  = 1.5;
wB_dk_phi = 0.1;
wB_dk_psi = 0.01;

% Matrice di peso per le prestazioni, sulla S
wP_dk_phi = 1/100 * (s/M_dk + wB_dk_phi)/(s + wB_dk_phi*A_dk);
wP_dk_psi = 1/100 * (s/M_dk + wB_dk_psi)/(s + wB_dk_psi*A_dk);
WP = blkdiag(wP_dk_phi, wP_dk_psi); % Matrice diagonale di peso per S

%% Generalized plant P by using 

G.u = 'uG';
G.y = 'yG';

WP.u = 'e';
WP.y = 'z1';

WI.u = 'u';
WI.y = 'yd';

Sum1 = sumblk('y=w+yG', 2);
Sum2 = sumblk('uG=u+ud', 2);
Sum3 = sumblk('e=-y', 2);

P = connect(G, WI, WP, Sum1,Sum2,Sum3, {'ud','w','u'},{'yd','z1','e'});


%% D-K iteration

% Inizializzazione
blk = [1 1;1 1;2 2];
nmeas = 2; 
nu = 2; 
d0 = 1;
D = append(d0, d0, tf(eye(2)), tf(eye(2)));

figure;
hold on;
grid on;

legendEntries = {};

for k = 1:2

% STEP 1: trovare un controllore ottimo H infinito
[K, Nsc, gamma, info] = hinfsyn(D*P/D, nmeas, nu,'method','lmi','Tolgam', 1e-3);

Nf = frd(lft(P,K),omega);

% STEP 2: calcolare mu usando l'upper bound
[mubnds,Info] = mussv(Nf, blk);
bodemag(mubnds(1,1), omega);
murp = norm(mubnds(1,1), inf,1e-6);

legendEntries{end+1} = sprintf('\\mu al passo %d', k); 

% STEP 3: fit della D ricavata
[VDelta,VSigma] = mussvextract(Info);
VSigma.DLeft = VSigma.DLeft/VSigma.DLeft(3,3);
d1 = fitmagfrd(VSigma.DLeft(1,1),4);              % fit 4th ordine a fase minima
hold on
grid on

% Ritorn allo STEP 1 
D = append(d1,d1,tf(eye(2)),tf(eye(2)));

end

K_DK_tf = tf(K);
legend(legendEntries, 'Location', 'Best');
