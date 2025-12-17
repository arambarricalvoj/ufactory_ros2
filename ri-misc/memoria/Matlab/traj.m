clear; clc; close all;
syms q1 q2 q3 q4 q5 q6 a2 T2_offset T3_offset real
pi2 = sym(pi)/2;

%% ================================
%  Definición de eslabones (DH modificado)
% ================================
L1 = Link([0     267    0      0        0  0], 'modified');
L2 = Link([0     0      0     -pi2      0  T2_offset], 'modified');
L3 = Link([0     0      a2     0        0  T3_offset], 'modified');
L4 = Link([0     342.5  77.5  -pi2      0  0], 'modified');
L5 = Link([0     0      0      pi2      0  0], 'modified');
L6 = Link([0     97     76    -pi2      0  0], 'modified');

%% ================================
%  Transformación completa T0^6
% ================================
T01 = L1.A(q1).T; T12 = L2.A(q2).T; T23 = L3.A(q3).T;
T34 = L4.A(q4).T; T45 = L5.A(q5).T; T56 = L6.A(q6).T;
T06 = simplify(T01 * T12 * T23 * T34 * T45 * T56);
p06 = T06(1:3,4);

% Funciones numéricas
T_fun = matlabFunction(T06, 'Vars', {q1,q2,q3,q4,q5,q6,a2,T2_offset,T3_offset});

%% ================================
%  Parámetros numéricos del robot
% ================================
a2_val        = 289.48866;
T2_offset_val = deg2rad(-79.34995);
T3_offset_val = deg2rad(79.34995);

% Límites articulares (matemáticos, en grados)
q1_min = -360;     q1_max =  360;
q2_min = -117 - rad2deg(T2_offset_val);
q2_max =  116 - rad2deg(T2_offset_val);
q3_min = -219 - rad2deg(T3_offset_val);
q3_max =   10 - rad2deg(T3_offset_val);
q4_min = -360;     q4_max =  360;
q5_min = -97;      q5_max =  180;
q6_min = -360;     q6_max =  360;

% Convertir a radianes
q_min = deg2rad([q1_min q2_min q3_min q4_min q5_min q6_min]);
q_max = deg2rad([q1_max q2_max q3_max q4_max q5_max q6_max]);

%% ================================
%  Trayectoria articular (quintic)
% ================================
q0 = zeros(1,6); % inicio en 0
% finales aleatorios dentro de rango
qf = q_min + rand(1,6).*(q_max - q_min);

T = 5; % duración nominal inicial
n = 200; t = linspace(0,T,n);

% Polinomio quíntico (vel y acc inicial/final = 0)
Q = zeros(n,6);
for i=1:6
    a0 = q0(i); a1 = 0; a2 = 0;
    a3 = 10*(qf(i)-q0(i))/T^3;
    a4 = -15*(qf(i)-q0(i))/T^4;
    a5 = 6*(qf(i)-q0(i))/T^5;
    Q(:,i) = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
end

%% ================================
%  IPTP: ajuste temporal
% ================================
% Límites dinámicos (ejemplo, ajusta según datasheet)
vel_max = deg2rad([180 180 180 180 180 180]);   % rad/s
acc_max = deg2rad([1500 1500 1500 1500 1500 1500]);   % rad/s^2

dt = t(2)-t(1);
dQ = diff(Q)/dt;          % velocidades
ddQ = diff(dQ)/dt;        % aceleraciones

% Escalado temporal si excede límites
scale_v = max(max(abs(dQ)./vel_max));
scale_a = max(max(abs(ddQ)./acc_max));
scale = max([1, scale_v, sqrt(scale_a)]);

if scale > 1
    T = T*scale; % estira el tiempo total
    t = linspace(0,T,n);
    for i=1:6
        a0 = q0(i); a1 = 0; a2 = 0;
        a3 = 10*(qf(i)-q0(i))/T^3;
        a4 = -15*(qf(i)-q0(i))/T^4;
        a5 = 6*(qf(i)-q0(i))/T^5;
        Q(:,i) = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
    end
end

%% ================================
%  Visualización de articulaciones
% ================================
figure;
plot(t, rad2deg(Q));
xlabel('Tiempo [s]'); ylabel('Ángulo [°]');
legend('q1','q2','q3','q4','q5','q6');
title('Trayectoria articular (quíntica con IPTP)');

%% ================================
%  Trayectoria del efector final
% ================================
P = zeros(n,3);
for k=1:n
    Tnow = T_fun(Q(k,1),Q(k,2),Q(k,3),Q(k,4),Q(k,5),Q(k,6), ...
                 a2_val,T2_offset_val,T3_offset_val);
    P(k,:) = Tnow(1:3,4).';
end

figure;
plot3(P(:,1),P(:,2),P(:,3),'b-','LineWidth',2);
grid on; xlabel('X'); ylabel('Y'); zlabel('Z');
title('Trayectoria del efector final (quíntica con IPTP)');

%% ================================
%  Imprimir posiciones inicial y final
% ================================
% Valores físicos (sumando offset en q2 y q3)
q0_phys = q0;
q0_phys(2) = q0_phys(2) + T2_offset_val;
q0_phys(3) = q0_phys(3) + T3_offset_val;

qf_phys = qf;
qf_phys(2) = qf_phys(2) + T2_offset_val;
qf_phys(3) = qf_phys(3) + T3_offset_val;

disp('=== Valores iniciales físicos (grados) ===');
disp(rad2deg(q0_phys));

disp('=== Valores finales físicos (grados) ===');
disp(rad2deg(qf_phys));

