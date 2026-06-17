clc 
clear 
close all

Parameter_CSI;

a = A_tot_nom;
b = B_tot_nom;
c = C_tot_nom;
d = D_tot_nom;

Znm = zeros(n,m); 
Zmm = zeros(m,m);
Znn = zeros(n,n); 
Zmn = zeros(m,n);

%% 1) Design del controllore (regolatore) in feedback
% aumento l'impianto con gli integratori
Alqg = [a Znm;-c Zmm]; 
Blqg = [b;-d];             

Q = 1e-2 * [Znn Znm;Zmn eye(m,m)];                 % peso o errore integrato
R = 0.01 * eye(m);                                 % peso sull'ingresso

Kr = lqr(Alqg,Blqg,Q,R);                           % regolatore ottimo in feedback

% estraggo l'integratore e il feedback sullo stato
Krp = Kr(1:m,1:n); 
Kri = Kr(1:m,n+1:n+m);   

%% 2) Design del filtro di Kalman            
Bnoise = eye(n);                                            % modello del rumore di processo (Gd)
W = 20 * eye(size(Bnoise, 2));                                   % disturbo di processo
V = 0.1 * eye(p);                                               % rumore di misura
sys_tot_nom_noise = ss(a, [b Bnoise], c, [d zeros(p,n)]);
[kalmfss, Ke, P] = kalman(sys_tot_nom_noise, W, V);         % guadagno del filtro di Kalman


%% 3) Controllore da [r y]’ a u (2 DOF) e da -y a u (1 DOF)
%Considero anche gli integratori
Ac = [Zmm Zmn; -b*Kri a-b*Krp-Ke*c];           
Bcr = [eye(m); Znm]; 
Bcy=[-eye(m); Ke];
Cc = [-Kri -Krp]; 
Dcr = Zmm; 
Dcy = Zmm;
Klqg2 = ss(Ac,[Bcr Bcy],Cc,[Dcr Dcy]);      % Controllore a 2 g.d.l. da [r y]' a u
Klqg = ss(Ac,-Bcy,Cc,-Dcy);                 % Parte in feedback del controllore da -y a u
Klqg_tf = minreal(tf(Klqg));

%% Simulazione con il rumore di processo
% simulazione 1 g.d.l.
CL = feedback(G_tot_nom*Klqg, eye(p)); 

% simulazione 2 g.d.l.
CL2 = (feedback(G_tot_nom*Klqg2, eye(p), [3 4], [1 2], +1));  
sys2 = CL2;

%sforzo di controllo 2 g.d.l.
CL2u = feedback(Klqg2, G_tot_nom, [3 4], [1 2], +1); 
sys2u = CL2u; 

figure
step(CL(1,1));
hold on 
grid on
step(sys2(1,1));
hold on
legend('1 g.d.l.','2 g.d.l.')

figure
step(CL(2,2));
hold on 
grid on
step(sys2(2,2));
legend('1 g.d.l.','2 g.d.l.')



