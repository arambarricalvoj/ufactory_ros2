clear; clc; close all;
syms q1 q2 q3 q4 q5 q6 a2 T2_offset T3_offset real
pi2 = sym(pi)/2;

%% ================================
%  Modelo xArm6 (DH modificado)
% ================================
L1 = Link([0     267    0      0        0  0], 'modified');
L2 = Link([0     0      0     -pi2      0  T2_offset], 'modified');
L3 = Link([0     0      a2     0        0  T3_offset], 'modified');
L4 = Link([0     342.5  77.5  -pi2      0  0], 'modified');
L5 = Link([0     0      0      pi2      0  0], 'modified');
L6 = Link([0     97     76    -pi2      0  0], 'modified');

%% ================================
%  Transformaciones y Jacobiano
% ================================
T01 = L1.A(q1).T; T12 = L2.A(q2).T; T23 = L3.A(q3).T;
T34 = L4.A(q4).T; T45 = L5.A(q5).T; T56 = L6.A(q6).T;
T06 = simplify(T01 * T12 * T23 * T34 * T45 * T56);
p06 = T06(1:3,4);
Jv = jacobian(p06, [q1 q2 q3 q4 q5 q6]);

% Funciones numéricas
p_fun = matlabFunction(p06, 'Vars', {q1,q2,q3,q4,q5,q6,a2,T2_offset,T3_offset});
J_fun = matlabFunction(Jv, 'Vars', {q1,q2,q3,q4,q5,q6,a2,T2_offset,T3_offset});
T_fun = matlabFunction(T06, 'Vars', {q1,q2,q3,q4,q5,q6,a2,T2_offset,T3_offset});

%% ================================
%  Parámetros del robot
% ================================
a2_val        = 289.48866;
T2_offset_val = deg2rad(-79.34995);
T3_offset_val = deg2rad(79.34995);

% Límites físicos → matemáticos (deg)
q1_min = -360;     q1_max =  360;
q2_min = -117 - rad2deg(T2_offset_val);
q2_max =  116 - rad2deg(T2_offset_val);
q3_min = -219 - rad2deg(T3_offset_val);
q3_max =   10 - rad2deg(T3_offset_val);
q4_min = -360;     q4_max =  360;
q5_min = -97;      q5_max =  180;
q6_min = -360;     q6_max =  360;

q_min = deg2rad([q1_min, q2_min, q3_min, q4_min, q5_min, q6_min])';
q_max = deg2rad([q1_max, q2_max, q3_max, q4_max, q5_max, q6_max])';

%% ================================
%  Entrada: articulaciones actuales, pose final y duración
% ================================
dlg = inputdlg( ...
    {'Articulaciones actuales [q1 q2 q3 q4 q5 q6] en grados (matemático):', ...
     'Posición final del efector [x y z] en mm:', ...
     'Duración nominal de la trayectoria [s]:'}, ...
    'Entrada de movimiento', [1 70], {'0 0 0 0 0 0', '350 150 250', '5'});

if isempty(dlg)
    q0_deg = input('Introduce [q1 q2 q3 q4 q5 q6] en grados (matemático): ');
    p_goal = input('Introduce [x y z] en mm para la posición final del efector: ');
    T      = input('Introduce la duración nominal de la trayectoria [s]: ');
else
    q0_deg = str2num(dlg{1});
    p_goal = str2num(dlg{2}); 
    T      = str2double(dlg{3});
end

assert(numel(q0_deg)==6, 'Debes introducir 6 valores articulares.');
assert(numel(p_goal)==3, 'Debes introducir 3 valores cartesianas [x y z].');

% Convertir a radianes y columna
q_init = deg2rad(q0_deg(:));
p_goal = p_goal(:);

% Pose inicial cartesiana derivada de las articulaciones actuales
T_init = T_fun(q_init(1), q_init(2), q_init(3), q_init(4), q_init(5), q_init(6), ...
               a2_val, T2_offset_val, T3_offset_val);
p_init = T_init(1:3,4);

%% ================================
%  Verificación de alcanzabilidad (radio de acción)
% ================================
reach_max = 700; % mm (xArm6)
d_goal = norm(p_goal);

if d_goal > reach_max
    fprintf('Objetivo a %.1f mm, fuera del alcance máximo de %.1f mm -> NO realizable\n', ...
        d_goal, reach_max);
    return;   % detener el programa aquí
end

fprintf('Objetivo a %.1f mm, dentro del alcance máximo %.1f mm -> Realizable\n', ...
    d_goal, reach_max);

%% ================================
%  IK numérica (solo posición) con penalización de límites
%  (SIN clamping: se mantiene tu diseño original)
% ================================
tol = 1e-6; maxIter = 80; K = 0.005; % penalización
q_mid = (q_min + q_max)/2;

% Semilla: las articulaciones actuales
q_goal_seed = q_init;
q_goal = newtonIK(p_goal, q_goal_seed, p_fun, J_fun, ...
                  a2_val, T2_offset_val, T3_offset_val, ...
                  q_min, q_max, q_mid, K, tol, maxIter);

%% ================================
%  Trayectoria quíntica + IPTP
% ================================
n = 300; t = linspace(0,T,n);
Q = quinticTraj(q_init.', q_goal.', T, t);

% Límites dinámicos del xArm6 (usa valores del manual)
vel_max = deg2rad([180 180 180 180 180 180]);       % rad/s
acc_max = deg2rad([1500 1500 1500 1500 1500 1500]); % rad/s^2

% IPTP: escalar tiempo si excede límites (recalcula la quintic con T escalado)
[Q, t, T_scaled] = iptp_scale_time(Q, t, vel_max, acc_max);

%% ================================
%  Cálculo de velocidades y aceleraciones
% ================================
dt = t(2) - t(1);          % intervalo de muestreo
dQ = diff(Q) / dt;         % velocidades articulares (rad/s), tamaño (n-1) x 6
ddQ = diff(dQ) / dt;       % aceleraciones articulares (rad/s^2), tamaño (n-2) x 6

% Velocidad cartesiana del efector usando Jacobiano lineal
v_cart = zeros(n-1,3);
for k = 1:(n-1)
    J_now = J_fun(Q(k,1),Q(k,2),Q(k,3),Q(k,4),Q(k,5),Q(k,6), a2_val, T2_offset_val, T3_offset_val);
    v_cart(k,:) = (J_now * dQ(k,:)').'; % [vx vy vz] en mm/s si p_fun/T_fun están en mm
end

%% ================================
%  Impresión de resultados (físicos y matemáticos)
% ================================
q0      = q_init(:).';
qf      = q_goal(:).';

% Valores físicos (sumando offsets en q2 y q3)
q0_phys = q0; q0_phys(2) = q0_phys(2) + T2_offset_val; q0_phys(3) = q0_phys(3) + T3_offset_val;
qf_phys = qf; qf_phys(2) = qf_phys(2) + T2_offset_val; qf_phys(3) = qf_phys(3) + T3_offset_val;

fprintf('\n=== Estado ===\n');
fprintf('Duración planificada (IPTP): %.2f s\n', T_scaled);

disp('\n=== Articulaciones iniciales (matemático) [deg] ==='); disp(rad2deg(q0));
disp('=== Articulaciones finales (matemático) [deg]   ==='); disp(rad2deg(qf));
disp('=== Articulaciones iniciales (físico) [deg]     ==='); disp(rad2deg(q0_phys));
disp('=== Articulaciones finales (físico) [deg]       ==='); disp(rad2deg(qf_phys));
disp('=== Velocidades articulares iniciales [deg/s] ==='); disp(rad2deg(dQ(1,:)));
disp('=== Velocidades articulares finales [deg/s] ==='); disp(rad2deg(dQ(end,:)));
disp('=== Aceleraciones articulares iniciales [deg/s^2] ==='); disp(rad2deg(ddQ(1,:)));
disp('=== Aceleraciones articulares finales [deg/s^2] ==='); disp(rad2deg(ddQ(end,:)));

%% ================================
%  Trayectoria del efector final
% ================================
P = zeros(n,3);
for k=1:n
    Tnow = T_fun(Q(k,1),Q(k,2),Q(k,3),Q(k,4),Q(k,5),Q(k,6), a2_val, T2_offset_val, T3_offset_val);
    P(k,:) = Tnow(1:3,4).';
end

%% ================================
%  Gráficas con anotaciones (posiciones físicas)
% ================================
% Añadir offsets a J2 y J3 para graficar valores físicos
Q_phys = Q;
Q_phys(:,2) = Q_phys(:,2) + T2_offset_val; % offset J2
Q_phys(:,3) = Q_phys(:,3) + T3_offset_val; % offset J3

figure;
plot(t, rad2deg(Q_phys), 'LineWidth',1.5); hold on;
xlabel('Tiempo [s]'); ylabel('Ángulo [°]');
legend('q1','q2','q3','q4','q5','q6','Location','bestoutside');
title('Trayectoria articular (quíntica + IPTP, valores físicos)');

% Puntos inicio/fin visibles pero EXCLUIDOS de la leyenda
for i=1:6
    plot(t(1), rad2deg(Q_phys(1,i)), 'go', 'MarkerFaceColor','g', 'HandleVisibility','off'); % inicio
    text(t(1), rad2deg(Q_phys(1,i)), sprintf('%.1f°', rad2deg(q0_phys(i))), ...
        'VerticalAlignment','bottom','HorizontalAlignment','right');
    plot(t(end), rad2deg(Q_phys(end,i)), 'ro', 'MarkerFaceColor','r', 'HandleVisibility','off'); % final
    text(t(end), rad2deg(Q_phys(end,i)), sprintf('%.1f°', rad2deg(qf_phys(i))), ...
        'VerticalAlignment','top','HorizontalAlignment','left');
end

%% ================================
%  Gráficas de velocidades y aceleraciones articulares
% ================================
% Nota: dQ es de longitud n-1 y ddQ de n-2
figure;
plot(t(1:end-1), rad2deg(dQ), 'LineWidth',1.2); grid on;
xlabel('Tiempo [s]'); ylabel('Velocidad [°/s]');
legend('q1','q2','q3','q4','q5','q6','Location','bestoutside');
title('Velocidades articulares');

figure;
plot(t(1:end-2), rad2deg(ddQ), 'LineWidth',1.2); grid on;
xlabel('Tiempo [s]'); ylabel('Aceleración [°/s^2]');
legend('q1','q2','q3','q4','q5','q6','Location','bestoutside');
title('Aceleraciones articulares');

%% ================================
%  Gráfica de velocidad y aceleración cartesiana del efector
% ================================
figure;
plot(t(1:end-1), v_cart, 'LineWidth',1.2); grid on;
xlabel('Tiempo [s]'); ylabel('Velocidad lineal [mm/s]');
legend('v_x','v_y','v_z','Location','bestoutside');
title('Velocidad cartesiana del efector');

a_cart = diff(v_cart) / dt;       % aceleración cartesiana (mm/s^2), ...
%     tamaño (n-2) x 3
figure;
plot(t(1:end-2), a_cart, 'LineWidth',1.2); grid on;
xlabel('Tiempo [s]'); ylabel('Aceleración lineal [mm/s^2]');
legend('a_x','a_y','a_z','Location','bestoutside');
title('Aceleración cartesiana del efector');

%% ================================
%  Trayectoria del efector final (posición)
% ================================
figure;
plot3(P(:,1),P(:,2),P(:,3),'b-','LineWidth',2); hold on;
plot3(P(1,1),P(1,2),P(1,3),'go','MarkerFaceColor','g'); text(P(1,1),P(1,2),P(1,3),'Inicio', ... 
    'VerticalAlignment','bottom');
plot3(P(end,1),P(end,2),P(end,3),'ro','MarkerFaceColor','r'); text(P(end,1),P(end,2),P(end,3), ... 
    'Final','VerticalAlignment','top');
grid on; xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
title('Trayectoria del efector final');

%% ================================
%  Funciones auxiliares
% ================================
function q_sol = newtonIK(p_d, q_seed, p_fun, J_fun, a2_val, T2_offset_val, T3_offset_val, q_min, ...
    q_max, q_mid, K, tol, maxIter)
    % Newton–Raphson SIN clamping (solo penalización suave de límites)
    qk = q_seed(:);
    for it = 1:maxIter
        p_now = p_fun(qk(1), qk(2), qk(3), qk(4), qk(5), qk(6), a2_val, T2_offset_val, T3_offset_val);
        J_now = J_fun(qk(1), qk(2), qk(3), qk(4), qk(5), qk(6), a2_val, T2_offset_val, T3_offset_val);
        e = p_d - p_now;
        if norm(e) < tol, break; end
        gradH = 2 * (qk - q_mid) ./ ((q_max - q_min).^2);
        dq = pinv(J_now) * e - K * gradH;
        qk = qk + dq;
    end
    q_sol = qk;
end

function Q = quinticTraj(q0, qf, T, t)
    % Polinomio quíntico con vel y acc cero en extremos
    n = numel(t); Q = zeros(n, numel(q0));
    for i=1:numel(q0)
        a0 = q0(i); a1 = 0; a2 = 0;
        a3 = 10*(qf(i)-q0(i))/T^3;
        a4 = -15*(qf(i)-q0(i))/T^4;
        a5 = 6*(qf(i)-q0(i))/T^5;
        Q(:,i) = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
    end
end

function [Q_out, t_out, T_scaled] = iptp_scale_time(Q, t, vel_max, acc_max)
    % Escalado temporal para respetar límites de velocidad y aceleración
    dt = t(2) - t(1);
    dQ  = diff(Q)/dt;
    ddQ = diff(dQ)/dt;
    scale_v = max(max(abs(dQ) ./ vel_max));
    scale_a = max(max(abs(ddQ) ./ acc_max));
    scale = max([1, scale_v, sqrt(scale_a)]);
    if scale > 1
        T_scaled = t(end) * scale;
        t_out = linspace(0, T_scaled, numel(t));
        % Recalcular la quintic con el nuevo T para mantener vel/acc cero en extremos
        Q_out = zeros(size(Q));
        q0 = Q(1,:); qf = Q(end,:);
        for i=1:size(Q,2)
            a0 = q0(i); a1 = 0; a2 = 0;
            a3 = 10*(qf(i)-q0(i))/T_scaled^3;
            a4 = -15*(qf(i)-q0(i))/T_scaled^4;
            a5 = 6*(qf(i)-q0(i))/T_scaled^5;
            Q_out(:,i) = a0 + a1*t_out + a2*t_out.^2 + a3*t_out.^3 + a4*t_out.^4 + a5*t_out.^5;
        end
    else
        Q_out = Q; t_out = t; T_scaled = t(end);
    end
end

