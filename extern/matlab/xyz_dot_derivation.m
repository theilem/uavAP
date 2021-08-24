syms psi_1 theta_1 phi_1;
syms u_1 v_1 w_1;

Rz = [cos(psi_1), -sin(psi_1), 0; sin(psi_1), cos(psi_1), 0; 0, 0, 1];
Ry = [cos(theta_1), 0, sin(theta_1); 0, 1, 0; -sin(theta_1), 0, cos(theta_1)];
Rx = [1, 0, 0; 0, cos(phi_1), -sin(phi_1); 0, sin(phi_1), cos(phi_1)];
vel = [u_1; v_1; w_1];

xyz_dot = Rz * Ry * Rx * vel;

vx = xyz_dot(1);
vy = xyz_dot(2);
vz = -xyz_dot(3);

x_u = diff(vx, u_1);
x_v = diff(vx, v_1);
x_w = diff(vx, w_1);

y_u = diff(vy, u_1);
y_v = diff(vy, v_1);
y_w = diff(vy, w_1);

z_u = diff(vz, u_1);
z_v = diff(vz, v_1);
z_w = diff(vz, w_1);

x_phi = diff(vx, phi_1);
x_theta = diff(vx, theta_1);
x_psi = diff(vx, psi_1);

y_phi = diff(vy, phi_1);
y_theta = diff(vy, theta_1);
y_psi = diff(vy, psi_1);

z_phi = diff(vz, phi_1);
z_theta = diff(vz, theta_1);
z_psi = diff(vz, psi_1);