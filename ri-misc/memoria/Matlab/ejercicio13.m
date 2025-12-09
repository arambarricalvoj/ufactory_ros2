% SIMBÓLICO
syms phi theta psi real

% Definir las matrices simbólicas de rotación
Rz1 = [cos(phi) -sin(phi) 0;
       sin(phi)  cos(phi) 0;
       0            0           1];

Ry  = [cos(theta)  0 sin(theta);
       0            1 0;
      -sin(theta)  0 cos(theta)];

Rz2 = [cos(psi) -sin(psi) 0;
       sin(psi)  cos(psi) 0;
       0            0           1];

% Composición simbólica
R = simplify(Rz1 * Ry * Rz2);
disp(R)

% Matriz homogénea simbólica
T = [R, [0;0;0]; 0 0 0 1];


% NUMÉRICO
phi = deg2rad(45);
theta = deg2rad(90);
psi = deg2rad(12);

Rz1 = [cos(phi) -sin(phi) 0;
       sin(phi)  cos(phi) 0;
       0            0           1];

Ry  = [cos(theta)  0 sin(theta);
       0            1 0;
      -sin(theta)  0 cos(theta)];

Rz2 = [cos(psi) -sin(psi) 0;
       sin(psi)  cos(psi) 0;
       0            0           1];

R = Rz1 * Ry * Rz2;
T = [R, [0;0;0]; 0 0 0 1];
disp(T)



