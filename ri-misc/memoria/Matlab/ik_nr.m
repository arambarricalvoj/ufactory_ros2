clear; clc;
syms q1 q2 q3 q4 q5 q6 a2 T2_offset T3_offset real
pi2 = sym(pi)/2;

% Definición de eslabones según la tabla DH modificada
L1 = Link([0     267    0      0        0  0], 'modified');        % Eslabón 1
L2 = Link([0     0      0     -pi2      0  T2_offset], 'modified');% Eslabón 2
L3 = Link([0     0      a2     0        0  T3_offset], 'modified');% Eslabón 3
L4 = Link([0     342.5  77.5  -pi2      0  0], 'modified');        % Eslabón 4
L5 = Link([0     0      0      pi2      0  0], 'modified');        % Eslabón 5
L6 = Link([0     97     76    -pi2      0  0], 'modified');        % Eslabón 6

% Matrices parciales
T01 = L1.A(q1).T;
T12 = L2.A(q2).T;
T23 = L3.A(q3).T;
T34 = L4.A(q4).T;
T45 = L5.A(q5).T;
T56 = L6.A(q6).T;

% Transformación completa
T06 = simplify(T01 * T12 * T23 * T34 * T45 * T56);
p06 = T06(1:3,4);

% Jacobiano lineal (para cinemática inversa)
Jv = jacobian(p06, [q1 q2 q3 q4 q5 q6]);

% Convertir a funciones numéricas
p_fun = matlabFunction(p06, 'Vars', ...
    {q1, q2, q3, q4, q5, q6, a2, T2_offset, T3_offset});
J_fun = matlabFunction(Jv, 'Vars', ...
    {q1, q2, q3, q4, q5, q6, a2, T2_offset, T3_offset});
T_fun = matlabFunction(T06, 'Vars', ...
    {q1, q2, q3, q4, q5, q6, a2, T2_offset, T3_offset});

% Parámetros numéricos del robot
a2_val        = 289.48866;
T2_offset_val = deg2rad(-79.34995);
T3_offset_val = deg2rad(79.34995);

% ---
% Cinemática inversa numérica (NEWTON–RAPHSON)
% ---
% Posición deseada (se sabe que corresponde a q_i = 0 con esos offsets)
p_d = [207; 0; 112];

% Configuración inicial
qk = zeros(6,1);   % punto de partida
tol     = 1e-6;
maxIter = 50;

% Límites matemáticos de las articulaciones (incluyendo offsets en 2 y 3)
% Rangos físicos (tabla 1):
% J1:  ±360º
% J2:  -117º -- 116º  (físicos)
% J3:  -219º -- 10º   (físicos)
% J4:  ±360º
% J5:  -97º -- 180º
% J6:  ±360º
%
% Convertimos a límites "matemáticos" para q2 y q3 (sin offset):
% q2_mat ∈ [q2_fis_min - offset2, q2_fis_max - offset2]
% q3_mat ∈ [q3_fis_min - offset3, q3_fis_max - offset3]
q1_min = -360;     q1_max =  360;
q2_min = -117 - rad2deg(T2_offset_val);   % grados matemáticos
q2_max =  116 - rad2deg(T2_offset_val);
q3_min = -219 - rad2deg(T3_offset_val);
q3_max =   10 - rad2deg(T3_offset_val);
q4_min = -360;     q4_max =  360;
q5_min = -97;      q5_max =  180;
q6_min = -360;     q6_max =  360;

q_min = deg2rad([q1_min, q2_min, q3_min, q4_min, q5_min, q6_min])';
q_max = deg2rad([q1_max, q2_max, q3_max, q4_max, q5_max, q6_max])';

% Centro del rango (para penalización suave)
q_mid = (q_min + q_max) / 2;

% Ganancia del término de penalización (joint limit avoidance)
K = 0.005;

for k = 1:maxIter
    
    % Evaluar posición y Jacobiano 
    % (incluyen offsets vía a2_val, T2_offset_val, T3_offset_val)
    p_now = p_fun(qk(1), qk(2), qk(3), qk(4), qk(5), qk(6), ...
                  a2_val, T2_offset_val, T3_offset_val);   % 3x1
    J_now = J_fun(qk(1), qk(2), qk(3), qk(4), qk(5), qk(6), ...
                  a2_val, T2_offset_val, T3_offset_val);   % 3x6
    
    % Error
    e = p_d - p_now;
    
    if norm(e) < tol
        break
    end
    
    % Gradiente de la función para evitar límites
    gradH = 2 * (qk - q_mid) ./ ((q_max - q_min).^2);
    
    % Paso Newton–Raphson (pseudoinversa) + penalización suave
    dq = pinv(J_now) * e - K * gradH;
    
    % Actualizar
    qk = qk + dq;
end

% Redondear solución para limpiar ruido numérico
q_solution = round(qk, 12);
% ---

% Verificación: evaluar T06(q*)
T_check = T_fun(q_solution(1), q_solution(2), q_solution(3), ...
                q_solution(4), q_solution(5), q_solution(6), ...
                a2_val, T2_offset_val, T3_offset_val);
p_check = T_check(1:3,4);

disp('=== SOLUCIÓN DE CINEMÁTICA INVERSA (en radianes) ===')
disp(q_solution.')

disp('=== SOLUCIÓN DE CINEMÁTICA INVERSA (en grados) ===')
disp(rad2deg(q_solution).')

% Si se quieren ver los ángulos FÍSICOS de las articulaciones 2 y 3:
q_phys = q_solution;
q_phys(2) = q_phys(2) + T2_offset_val;
q_phys(3) = q_phys(3) + T3_offset_val;

disp('=== ÁNGULOS FÍSICOS (q + offset) EN GRADOS ===')
disp(rad2deg(q_phys).')

disp('=== POSICIÓN OBTENIDA ===')
disp(p_check)

disp('=== POSICIÓN DESEADA ===')
disp(p_d)

disp('=== ERROR VECTORIAL ===')
disp(e)

final_error = norm(e);
disp('=== ERROR FINAL ===')
disp(final_error)

