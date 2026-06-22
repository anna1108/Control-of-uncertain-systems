function Dx = Dynamics_CSI(in)

l = 0.4;            %[m] distance from center of mass to front axle
L = 1;              %[m] wheelbase
d = 0.3;            %[m] front semi-axle
vx = 1;             %[m/s] equilibrium velocity along x

R_rear = 0.3;       %[m] rear wheel radius

R_front = 0.3;       %[m] front wheel radius

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

a_true = M_veh + M_rear + 2*M_front + 2*Iay/(R_front^2);
b_true = (1/2)*(M_veh*(l^2) + M_rear*(L^2) + 2*M_front*(d^2) + Igz + Ipz + 2*Iaz + 2*Iay*((d/R_front)^2));


% State definition
y = in(1);          % y coordinate
theta = in(2);      % vehicle angular position
phi_dot = in(3);    % rear wheel angular velocity
psi = in(4);        % steering angular position
psi_dot = in(5);    % steering angular velocity
tau_phi = in(6);    % driving torque
tau_psi = in(7);    % steering torque
% 
% theta = wrapToPi(theta);
% psi = wrapToPi(psi);

% steering fork angle: psi, rear wheel angle: phi
% Dynamics definition

y_dot = R_rear*phi_dot*sin(theta)*cos(psi);
theta_dot = -(R_rear/L)*phi_dot*sin(psi);
M = [a_true*(R_rear^2)*(cos(psi))^2+b_true*((R_rear/L)^2)*(sin(psi))^2+Ipy (-Ipz)*(R_rear/L)*sin(psi);(-Ipz)*(R_rear/L)*sin(psi) Ipz];
M_inv = inv(M);
C = [(b_true*(R_rear/L)^2-a_true*(R_rear^2))*phi_dot*psi_dot*sin(psi)*cos(psi)+c_phi*phi_dot; -Ipz*(R_rear/L)*phi_dot*psi_dot*cos(psi)+c_psi*psi_dot];
dot_dot = M\([tau_phi;tau_psi]-C);
phi_dot_dot = dot_dot(1);
psi_dot_dot = dot_dot(2);


Dx = [y_dot;theta_dot;phi_dot_dot;psi_dot;psi_dot_dot];