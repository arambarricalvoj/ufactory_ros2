% MANUALMENTE
% Cuaternión: [qw, qx, qy, qz]
q = [0.5, 0.5, 0.5, 0.5];
qw = q(1); qx = q(2); qy = q(3); qz = q(4);

% Matriz de rotación a partir del cuaternión
R = [1 - 2*qy^2 - 2*qz^2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw;
     2*qx*qy + 2*qz*qw,     1 - 2*qx^2 - 2*qz^2,     2*qy*qz - 2*qx*qw;
     2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx^2 - 2*qy^2];

disp('Matriz de rotación:');
disp(R);

% CON TOOLBOX
% Crear cuaternión unitario
q = UnitQuaternion(0.5, [0.5 0.5 0.5]);

% Obtener la matriz de rotación
R = q.R;

disp('Matriz de rotación:');
disp(R);

