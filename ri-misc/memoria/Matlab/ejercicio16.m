% Definir la matriz de rotación
R = [1, 0, 0;
     0, 1/sqrt(2), -1/sqrt(2);
     0, 1/sqrt(2),  1/sqrt(2)];

% Crear un cuaternión unitario directamente desde la matriz de rotación
q = UnitQuaternion(R);

% Mostrar el cuaternión
disp('Cuaternión (formato [s, v]):');
disp(q);

% Si quieres acceder a los componentes individuales:
q_scalar = q.s;     % Parte escalar (qw)
q_vector = q.v;     % Parte vectorial [qx, qy, qz]
