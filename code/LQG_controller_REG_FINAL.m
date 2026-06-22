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

%% 1) Feedback regulator design
% augment plant with integrators
Alqg = [a Znm;-c Zmm]; 
Blqg = [b;-d];             

Q = 1e-2 * [Znn Znm;Zmn eye(m,m)];                 % weight on integrated error
R = 0.01 * eye(m);                                 % weight on input

Kr = lqr(Alqg,Blqg,Q,R);                           % optimal feedback regulator

% extract integrator and state feedback gains
Krp = Kr(1:m,1:n); 
Kri = Kr(1:m,n+1:n+m);   

%% 2) Kalman filter design
Bnoise = eye(n);                                            % process noise model (Gd)
W = 20 * eye(size(Bnoise, 2));                                   % process disturbance
V = 0.1 * eye(p);                                               % measurement noise
sys_tot_nom_noise = ss(a, [b Bnoise], c, [d zeros(p,n)]);
[kalmfss, Ke, P] = kalman(sys_tot_nom_noise, W, V);         % Kalman filter gain


%% 3) Controller from [r y]’ to u (2 DOF) and from -y to u (1 DOF)
% Include integrators
Ac = [Zmm Zmn; -b*Kri a-b*Krp-Ke*c];           
Bcr = [eye(m); Znm]; 
Bcy=[-eye(m); Ke];
Cc = [-Kri -Krp]; 
Dcr = Zmm; 
Dcy = Zmm;
Klqg2 = ss(Ac,[Bcr Bcy],Cc,[Dcr Dcy]);      % 2-DOF controller from [r y]' to u
Klqg = ss(Ac,-Bcy,Cc,-Dcy);                 % feedback part of controller from -y to u
Klqg_tf = minreal(tf(Klqg));

%% Simulation with process noise
% 1-DOF simulation
CL = feedback(G_tot_nom*Klqg, eye(p));

% 2-DOF simulation
CL2 = (feedback(G_tot_nom*Klqg2, eye(p), [3 4], [1 2], +1));  
sys2 = CL2;

% 2-DOF control effort
CL2u = feedback(Klqg2, G_tot_nom, [3 4], [1 2], +1); 
sys2u = CL2u; 

figure
step(CL(1,1));
hold on 
grid on
step(sys2(1,1));
hold on
legend('1 DOF','2 DOF')

figure
step(CL(2,2));
hold on 
grid on
step(sys2(2,2));
legend('1 DOF','2 DOF')



