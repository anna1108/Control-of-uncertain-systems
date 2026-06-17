clc
clear 
close all

%% Definizione dei parametri del sistema
l = 0.4;            %[m] distanza centro di massa dall'asse anteriore
L = 1;              %[m] interasse
d = 0.3;            %[m] semiasse anteriore
vx = 1;             %[m/s] velocità all'equilibrio su x

Rp_nom = 0.3;       %[m] raggio ruota posteriore
Rp = ureal('Rp', Rp_nom, 'Percentage', 5);

Ra_nom = 0.3;       %[m] raggio ruote anteriori
Ra = ureal('Ra', Ra_nom, 'Percentage', 5);

% Rp_nom = 0.3;       %[m] raggio ruota posteriore
% Rp = ureal('Rp', Rp_nom, 'Percentage', 1);
% 
% Ra_nom = 0.3;       %[m] raggio ruote anteriori
% Ra = ureal('Ra', Ra_nom, 'Percentage', 1);

Mv = 15;            %[kg] massa 
Mp = 0.5;           %[kg] massa ruota posteriore
Ma = 0.5;           %[kg] massa ruote anteriori

Igz = 0.655;        %[kg*m^2] momento inerzia asse z del veicolo reale
Ipz = 0.23;         %[kg*m^2] momento inerzia sterzo ruota posteriore asse z
Ipy = 0.15;         %[kg*m^2] momento inerzia ruota posteriore (asse rotazione)
Iay = 0.12;         %[kg*m^2] momento inerzia ruota anteriore (asse rotazione)
Iaz = 0.8;          %[kg*m^2] momento inerzia ruota anteriore asse z

c_phi = 0.01;       %coefficiente di attrito
c_psi = 0.02;       %coefficiente di attrito

a = Mv + Mp + 2*Ma + 2*Iay/(Ra^2);
b = (1/2)*(Mv*(l^2) + Mp*(L^2) + 2*Ma*(d^2) + Igz + Ipz + 2*Iaz + 2*Iay*((d/Ra)^2));

%% Definizione dei parametri degli attuatori
K_m1_nom = 1.08;
K_m1 = ureal('K_m1', K_m1_nom, 'Percentage', 10);

T_m1_nom = 0.005;
T_m1 = ureal('T_m1', T_m1_nom, 'Percentage', 20);

K_m2_nom = 0.335;
K_m2 = ureal('K_m2', K_m2_nom, 'Percentage', 10);

T_m2_nom = 0.002;
T_m2 = ureal('T_m2', T_m2_nom, 'Percentage', 20);

% K_m1_nom = 1.08;
% K_m1 = ureal('K_m1', K_m1_nom, 'Percentage', 2);
% 
% T_m1_nom = 0.005;
% T_m1 = ureal('T_m1', T_m1_nom, 'Percentage', 2);
% 
% K_m2_nom = 0.335;
% K_m2 = ureal('K_m2', K_m2_nom, 'Percentage', 2);
% 
% T_m2_nom = 0.002;
% T_m2 = ureal('T_m2', T_m2_nom, 'Percentage', 2);

T1_nom = T_m1_nom/2;
T1 = T1_nom + 0.05*T1_nom; %negli impianti perturbati modello il ritardo come ritardo massimo

omega = logspace(-3, 3, 100);

%% Definisco incertezze degli attuatori e modelli degli attuatori incerti

orderWt1 = 2; % Ordine dei pesi

% PHI
G_tau_phi =  minreal(tf(K_m2, [T_m2^2/100 1.01*T_m2 1]));
G_tau_phi_nom = minreal(G_tau_phi.NominalValue);
G_tau_phi_nom_tf = minreal(tf(G_tau_phi_nom));

%sample di G_tau_phi_array
rng('default'); 
G_tau_phi_array = usample(G_tau_phi, 100); 

G_tau_phi_array_frd = frd(G_tau_phi_array, omega);
[usys_tau_phi, info_tau_phi] = ucover(G_tau_phi_array_frd, G_tau_phi_nom, orderWt1, 'InputMult');

WI_tau_phi_frd = info_tau_phi.W1opt;
WI_tau_phi = tf(info_tau_phi.W1); % Peso attuatore tau_phi

% PSI
G_tau_psi =  minreal(tf(K_m1, [T_m1 1], 'InputDelay', T1));
%T1 non è un parametro incerto, ma è il valore massimo che il ritardo può assumere
G_tau_psi_nom = minreal(tf(K_m1_nom, [T_m1_nom 1]));
G_tau_psi_nom_tf = minreal(tf(G_tau_psi_nom));

%sample di G_tau_psi
rng('default'); %rng mi permette di prendere i sample nello stesso modo
G_tau_psi_array = usample(G_tau_psi, 100); %vettore in cui ogni elemento è un sample

G_tau_psi_array_frd = frd(G_tau_psi_array, omega);
[usys_tau_psi, info_tau_psi] = ucover(G_tau_psi_array_frd, G_tau_psi_nom, orderWt1, 'InputMult');
%In uscita ho il modello incerto usys e le informazioni riguardanti il fit

WI_tau_psi_frd = info_tau_psi.W1opt;
WI_tau_psi = tf(info_tau_psi.W1); % Peso attuatore tau_psi

% Matrice di peso complessiva con i WI sulla diagonale
WI_tot = blkdiag(WI_tau_phi, WI_tau_psi);

%Funzioni di trasferimento nominali degli attuatori in frequenza
G_tau_psi_nom_frd = frd(G_tau_psi_nom, omega);
G_tau_phi_nom_frd = frd(G_tau_phi_nom, omega);

% Definizione fdt attuatori reali 

delta_tau_psi = ultidyn('Delta', [1 1]);
delta_tau_phi = ultidyn('Delta', [1 1]);

G_tau_psi_per = G_tau_psi_nom * (1 + WI_tau_psi * delta_tau_psi);
G_tau_phi_per = G_tau_phi_nom * (1 + WI_tau_phi * delta_tau_phi);
G_act = blkdiag(G_tau_phi_per, G_tau_psi_per); %sys attuatori perturbati
G_act_nom = G_act.NominalValue; % sys attuatori nominale

% Definizione matrici del linearizzato

A = [0 vx 0 0 0;
     0 0 0 -vx/L 0;
     0 0 -c_phi/(a*Rp^2 + Ipy) 0 0;
     0 0 0 0 1;
     0 0 0 0 -(c_psi-(Ipz*vx)/L)/Ipz];

B = [0 0;
     0 0;
     1/(a*Rp^2 + Ipy) 0;
     0 0;
     0 1/Ipz];

C = [1 0 0 0 0;0 0 1 0 0]; % osservo y e phi_dot
D = zeros(2,2);

%% Analisi di raggiungibilità
R = ctrb(A.NominalValue, B.NominalValue); %restituisce la matrice di raggiungibilità R_sys_nom
rank_R = rank(R);
if rank_R == size(A.NominalValue,1)
    disp('Modello completamente raggiungibile')
else
    disp('Modello NON completamente raggiungibile')
end

%% Analisi osservabilità del modello nominale degli attuatori
O = obsv(A.NominalValue, C); %restituisce la matrice di osservabilità O_sys_nom
rank_O = rank(O);
if rank_O == size(A.NominalValue,1)
    disp('Modello completamente osservabile')
else
    disp('Modello NON completamente osservabile')
end

% Definizione sistema incerto e nominale

sys   = (ss(A,B,C,D));
G_sys = (tf(sys));
[M,Delta] = lftdata(sys);

Anom = A.NominalValue;
Bnom = B.NominalValue;

sys_nom = (ss(Anom, Bnom, C, D));
G_sys_nom = (tf(sys_nom));
poli_sys = pole(G_sys_nom);
zeri_sys = tzero(G_sys_nom);

%% Definizione sistema in catena diretta attuatori e sistema nominale
G_tot_nom = (G_sys_nom*tf(G_act_nom)); % fdt sys completo nominale
G_tot = G_sys*tf(G_act); % fdt sys completo perturbato
sys_tot_nom = minreal(ss(G_tot_nom));
A_tot_nom = sys_tot_nom.A;
B_tot_nom = sys_tot_nom.B;
C_tot_nom = sys_tot_nom.C;
D_tot_nom = sys_tot_nom.D;

% Ricavo le dimensioni del sistema nominale complessivo
[n, m] = size(B_tot_nom);
p = size(C_tot_nom,1);

G_tot_ss = G_act*sys; % ss del modello complessivo perturbato
[M_tot Delta_tot] = lftdata(G_tot_ss);

u_eq = [c_phi*vx/Rp_nom; 0];
x_eq = [0; 0; vx/Rp_nom; 0; 0];