syms phi theta psi real  % ángulos simbólicos

% Rotación sobre eje X
Rx = [1      0           0;
      0 cos(psi) -sin(psi);
      0 sin(psi)  cos(psi)];

% Rotación sobre eje Y
Ry = [cos(theta)  0 sin(theta);
      0           1 0;
     -sin(theta)  0 cos(theta)];

% Rotación sobre eje Z
Rz = [cos(phi) -sin(phi) 0;
      sin(phi)  cos(phi) 0;
      0         0        1];

% Composición: RotZ * RotY * RotX
R = simplify(Rz * Ry * Rx);

% Matriz homogénea simbólica
T = [R, [0;0;0]; 0 0 0 1]

% OBTENER ÁNGULOS DE EULER XYZ
R = [ 0.3536  -0.5732   0.7392;
      0.6124   0.7392   0.2803;
     -0.7071   0.3536   0.6124 ];
T = [R, [0;0;0]; 0 0 0 1];
eul = tr2rpy(T)  % devuelve rpy en radianes
eul_deg = round(rad2deg(eul))


