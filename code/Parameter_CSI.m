clc
clear 
close all

%% System parameters
l = 0.4;            %[m] distance from center of mass to front axle
L = 1;              %[m] wheelbase
d = 0.3;            %[m] front semi-axle
vx = 1;             %[m/s] equilibrium velocity along x

R_rear_nom = 0.3;       %[m] rear wheel radius
Rp_nom = R_rear_nom;    %[m] alias expected by Simulink models
R_rear = ureal('R_rear', R_rear_nom, 'Percentage', 5);

R_front_nom = 0.3;       %[m] front wheel radius
R_front = ureal('R_front', R_front_nom, 'Percentage', 5);

% R_rear_nom = 0.3;       %[m] rear wheel radius
% R_rear = ureal('R_rear', R_rear_nom, 'Percentage', 1);
%
% R_front_nom = 0.3;       %[m] front wheel radius
% R_front = ureal('R_front', R_front_nom, 'Percentage', 1);

M_veh = 15;            %[kg] vehicle body mass
M_rear = 0.5;           %[kg] rear wheel mass
M_front = 0.5;           %[kg] front wheel mass

Igz = 0.655;        %[kg*m^2] moment of inertia about z-axis of real vehicle
Ipz = 0.23;         %[kg*m^2] moment of inertia of rear wheel steering about z-axis
Ipy = 0.15;         %[kg*m^2] moment of inertia of rear wheel (rotation axis)
Iay = 0.12;         %[kg*m^2] moment of inertia of front wheel (rotation axis)
Iaz = 0.8;          %[kg*m^2] moment of inertia of front wheel about z-axis

c_phi = 0.01;       %friction coefficient
c_psi = 0.02;       %friction coefficient

a = M_veh + M_rear + 2*M_front + 2*Iay/(R_front^2);
b = (1/2)*(M_veh*(l^2) + M_rear*(L^2) + 2*M_front*(d^2) + Igz + Ipz + 2*Iaz + 2*Iay*((d/R_front)^2));

%% Actuator parameters
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
T1 = T1_nom + 0.05*T1_nom; % in perturbed plants the delay is modeled as the maximum possible delay

omega = logspace(-3, 3, 100);

%% Actuator uncertainty and uncertain actuator models

orderWt1 = 2; % Weight order

% PHI
G_tau_phi =  minreal(tf(K_m2, [T_m2^2/100 1.01*T_m2 1]));
G_tau_phi_nom = minreal(G_tau_phi.NominalValue);
G_tau_phi_nom_tf = minreal(tf(G_tau_phi_nom));

% Samples of G_tau_phi_array
rng('default'); 
G_tau_phi_array = usample(G_tau_phi, 100); 

G_tau_phi_array_frd = frd(G_tau_phi_array, omega);
[usys_tau_phi, info_tau_phi] = ucover(G_tau_phi_array_frd, G_tau_phi_nom, orderWt1, 'InputMult');

WI_tau_phi_frd = info_tau_phi.W1opt;
WI_tau_phi = tf(info_tau_phi.W1); % Actuator weight for tau_phi

% PSI
G_tau_psi =  minreal(tf(K_m1, [T_m1 1], 'InputDelay', T1));
% T1 is not uncertain, but represents the maximum possible delay value
G_tau_psi_nom = minreal(tf(K_m1_nom, [T_m1_nom 1]));
G_tau_psi_nom_tf = minreal(tf(G_tau_psi_nom));

% Samples of G_tau_psi
rng('default'); % rng ensures reproducible samples
G_tau_psi_array = usample(G_tau_psi, 100); % vector where each element is a sample

G_tau_psi_array_frd = frd(G_tau_psi_array, omega);
[usys_tau_psi, info_tau_psi] = ucover(G_tau_psi_array_frd, G_tau_psi_nom, orderWt1, 'InputMult');
% Output: uncertain model and fit information

WI_tau_psi_frd = info_tau_psi.W1opt;
WI_tau_psi = tf(info_tau_psi.W1); % Actuator weight for tau_psi

% Overall weight matrix with WI on the diagonal
WI_tot = blkdiag(WI_tau_phi, WI_tau_psi);

% Nominal actuator transfer functions in frequency domain
G_tau_psi_nom_frd = frd(G_tau_psi_nom, omega);
G_tau_phi_nom_frd = frd(G_tau_phi_nom, omega);

% Real actuator transfer function definition

delta_tau_psi = ultidyn('Delta', [1 1]);
delta_tau_phi = ultidyn('Delta', [1 1]);

G_tau_psi_per = G_tau_psi_nom * (1 + WI_tau_psi * delta_tau_psi);
G_tau_phi_per = G_tau_phi_nom * (1 + WI_tau_phi * delta_tau_phi);
G_act = blkdiag(G_tau_phi_per, G_tau_psi_per); % perturbed actuator system
G_act_nom = G_act.NominalValue; % nominal actuator system

% Linearized model matrices

A = [0 vx 0 0 0;
     0 0 0 -vx/L 0;
     0 0 -c_phi/(a*R_rear^2 + Ipy) 0 0;
     0 0 0 0 1;
     0 0 0 0 -(c_psi-(Ipz*vx)/L)/Ipz];

B = [0 0;
     0 0;
     1/(a*R_rear^2 + Ipy) 0;
     0 0;
     0 1/Ipz];

C = [1 0 0 0 0;0 0 1 0 0]; % observe y and phi_dot
D = zeros(2,2);

%% Reachability analysis
R = ctrb(A.NominalValue, B.NominalValue); % returns the reachability matrix
rank_R = rank(R);
if rank_R == size(A.NominalValue,1)
    disp('Model is fully reachable')
else
    disp('Model is NOT fully reachable')
end

%% Observability analysis of the nominal actuator model
O = obsv(A.NominalValue, C); % returns the observability matrix
rank_O = rank(O);
if rank_O == size(A.NominalValue,1)
    disp('Model is fully observable')
else
    disp('Model is NOT fully observable')
end

% Uncertain and nominal system definition

sys   = (ss(A,B,C,D));
G_sys = (tf(sys));
[M,Delta] = lftdata(sys);

Anom = A.NominalValue;
Bnom = B.NominalValue;

sys_nom = (ss(Anom, Bnom, C, D));
G_sys_nom = (tf(sys_nom));
poles_sys = pole(G_sys_nom);
zeros_sys = tzero(G_sys_nom);

%% Forward-path system: actuators and nominal system
G_tot_nom = (G_sys_nom*tf(G_act_nom)); % nominal overall transfer function
G_tot = G_sys*tf(G_act); % perturbed overall transfer function
sys_tot_nom = minreal(ss(G_tot_nom));
A_tot_nom = sys_tot_nom.A;
B_tot_nom = sys_tot_nom.B;
C_tot_nom = sys_tot_nom.C;
D_tot_nom = sys_tot_nom.D;

% Extract dimensions of the overall nominal system
[n, m] = size(B_tot_nom);
p = size(C_tot_nom,1);

G_tot_ss = G_act*sys; % state-space of the perturbed overall model
[M_tot Delta_tot] = lftdata(G_tot_ss);

u_eq = [c_phi*vx/R_rear_nom; 0];
x_eq = [0; 0; vx/R_rear_nom; 0; 0];
