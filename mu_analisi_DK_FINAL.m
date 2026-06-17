clear all
close all
clc

D_K_controller2_FINAL;
close all

s = tf('s');

%% Nominal model

G = (sys*G_act);

%% Uncertainty weight 

WI = WI_tot;

%% Performance weight
% Definizione dei parametri per le funzioni peso
A  = 1e-4;
M  = 1.5;
wB_phi = 0.1;
wB_psi = 0.01;

% Matrice di peso per le prestazioni, sulla S
wP_phi = 1/100 * (s/M + wB_phi)/(s + wB_phi*A);
wP_psi = 1/100 * (s/M + wB_psi)/(s + wB_psi*A);
WP = blkdiag(wP_phi, wP_psi); % Matrice diagonale di peso per S

%% Generalized plant P 

G.u = 'u';
G.y = 'yG';

WP.u = 'e';
WP.y = 'z1';

Sum1 = sumblk('y=w+yG',2);
Sum3 = sumblk('e= -y',2);

P = connect(G,WP,Sum1,Sum3,...
    {'w','u'},{'z1','e'});

%% Controllore K Hinf
K_mu = tf(K);

%% Mu-analisys
% Tolgo fuori le incertezze
[P_mu,Delta_tot]=lftdata(P);
% Metto dentro il controllore
N = lft(P_mu,K_mu);
omega = logspace(-2,4,100);
% Valuto gli elementi di N in un vettore di frequenze definite da omega
Nfr = frd(N,omega);

%% Analisys of RP 
%% Computation of mu for RP
blk = [10 10;2 2]; % Struttura incertezza complessiva
[mubnds_RP,muinfo_RP] = mussv(Nfr,blk);
muRP = mubnds_RP(:,1);
[muRPinf,muRPw] = norm(muRP,inf);

%% Computation of mu for RS
blk = [10 10]; % Struttura incertezza vera
N11 = Nfr(1:10,1:10); % estraggo N11
[mubnds_RS,muinfo_RS] = mussv(N11,blk);
muRS = mubnds_RS(:,1);
[muRSinf,muRSw] = norm(muRS,inf);

%% Analisys of NP with mussv
Nnp = Nfr(11:12,11:12); % estraggo N22
[mubnds_NP,muinfo_NP] = mussv(Nnp,[2 2]);
muNP = mubnds_NP(:,1);
[muNPinf,muNPw] = norm(muNP,inf);

%% Plotting of results
bodemag(muRP,'', muRS,'--',muNP,'-.',omega)
legend('muRP','muRS','muNP')
title('Mu analisi controllore Hinf');
grid on