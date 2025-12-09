%% TRANSFORMACIONES CON MTH
% Rotación de -pi alrededor del eje X
Rx = trotx(-pi/2);

% Rotación de -pi alrededor del eje Z
Rz = trotz(-pi/2);

% Traslación de (-2, 6, 5)
Tt = transl(-2, 6, 5);

% Composición total
T = Tt * Rx * Rz;
disp(T) % Mostrar resultado

%% ROTACIONES CON CUATERNIONES
% Rotación de -pi/2 alrededor del eje X
q1 = UnitQuaternion.Rx(-pi/2);

% Rotación de -pi/2 alrededor del eje Z (del sistema móvil)
q2 = UnitQuaternion.Rz(-pi/2);

q12 = q1 * q2;  % primero Rx, luego Rz en el sistema rotado
disp(q12)

R = q.R;  % convertir a matriz de rotación 3x3
disp(R)


