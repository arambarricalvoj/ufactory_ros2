% CON LA TOOLBOX
% Crear el cuaternión
q = UnitQuaternion(cos(pi/4), (1/sqrt(3)) * sin(pi/4) * [1 1 1]);

% Convertir a matriz de rotación
R = q.R;

% Extraer ángulos de Euler en convención ZYZ (WVW)
eul = tr2eul(R);  % devuelve [W, V, W]

% Mostrar resultados en grados
disp('Ángulos de Euler [W phi , Y theta, W psi] en grados:');
disp(rad2deg(eul));

% MANUALMENTE
% Cuaternión: [qw, qx, qy, qz]
qw = cos(pi/4);
qv = (1/sqrt(3)) * sin(pi/4);
qx = qv; qy = qv; qz = qv;

% Matriz de rotación
R = [1 - 2*qy^2 - 2*qz^2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw;
     2*qx*qy + 2*qz*qw,     1 - 2*qx^2 - 2*qz^2,     2*qy*qz - 2*qx*qw;
     2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx^2 - 2*qy^2];

% Extraer ángulos de Euler en convención ZYZ (WVW)
theta = acos(R(3,3));
phi   = atan2(R(2,3), R(1,3));
psi   = atan2(R(3,2), -R(3,1));

% Mostrar en grados
disp('Ángulos de Euler ZYZ [W psi, Y theta, W phi] en grados:');
disp(rad2deg([psi, theta, phi]));
