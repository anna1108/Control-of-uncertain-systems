function Dx = Dinamica_CSI(in)

l = 0.4;            %[m] distanza centro di massa dall'asse anteriore
L = 1;              %[m] interasse
d = 0.3;            %[m] semiasse anteriore
vx = 1;             %[m/s] velocità all'equilibrio su x

Rp = 0.3;       %[m] raggio ruota posteriore

Ra = 0.3;       %[m] raggio ruote anteriori

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

a_true = Mv + Mp + 2*Ma + 2*Iay/(Ra^2);
b_true = (1/2)*(Mv*(l^2) + Mp*(L^2) + 2*Ma*(d^2) + Igz + Ipz + 2*Iaz + 2*Iay*((d/Ra)^2));


% Definizione stato
y = in(1);          %coordinata y
theta = in(2);      %posizione angolare veicolo
phi_dot = in(3);    %velocità rotazione ruota posteriore
psi = in(4);        %posizione angolare sterzo
psi_dot = in(5);    %velocità angolare sterzo
tau_phi = in(6);    %coppia motrice
tau_psi = in(7);    %coppia sterzante
% 
% theta = wrapToPi(theta);
% psi = wrapToPi(psi);

%forchetta è psi, ricciolo è phi
% Definizione dinamica

y_dot = Rp*phi_dot*sin(theta)*cos(psi);
theta_dot = -(Rp/L)*phi_dot*sin(psi);
M = [a_true*(Rp^2)*(cos(psi))^2+b_true*((Rp/L)^2)*(sin(psi))^2+Ipy (-Ipz)*(Rp/L)*sin(psi);(-Ipz)*(Rp/L)*sin(psi) Ipz];
M_inv = inv(M);
C = [(b_true*(Rp/L)^2-a_true*(Rp^2))*phi_dot*psi_dot*sin(psi)*cos(psi)+c_phi*phi_dot; -Ipz*(Rp/L)*phi_dot*psi_dot*cos(psi)+c_psi*psi_dot];
dot_dot = M\([tau_phi;tau_psi]-C);
phi_dot_dot = dot_dot(1);
psi_dot_dot = dot_dot(2);


Dx = [y_dot;theta_dot;phi_dot_dot;psi_dot;psi_dot_dot];