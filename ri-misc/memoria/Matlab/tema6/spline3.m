clear all; close all; clc;

function [a,b,c,d] = cubicSplineCoeffs(qi, qf, dqi, dqf, T)
    % qi, qf: posiciones inicial y final
    % dqi, dqf: velocidades inicial y final
    % T: longitud del tramo (aquí T=1)
    a = qi;
    b = dqi;
    c = (3*(qf-qi)/T^2) - (2*dqi + dqf)/T;
    d = (-2*(qf-qi)/T^3) + (dqi + dqf)/T^2;
end

function [t_all, q_all] = evalSplineTramos(a1,b1,c1,d1,a2,b2,c2,d2)
    % Evalúa spline cúbico en dos tramos consecutivos [0,1] y [1,2]
    % Devuelve tiempo concatenado y posiciones concatenadas
    
    % Tramo 1
    t1 = linspace(0,1,100);
    q1 = a1 + b1*t1 + c1*t1.^2 + d1*t1.^3;
    
    % Tramo 2 (usar tau = t-1)
    t2 = linspace(1,2,100);
    tau = t2-1;
    q2 = a2 + b2*tau + c2*tau.^2 + d2*tau.^3;
    
    % Concatenar
    t_all = [t1 t2];
    q_all = [q1 q2];
end

% Nodos de tiempo
t = [0 1 2];

% Posiciones de cada eslabón
q1 = [90 45 0];      % grados
q2 = [1 2 4];        % metros
q3 = [0 22.5 45];    % grados
q4 = [4 3 2];        % metros

% Velocidades en nodos (Craig en t=1, extremos = 0)
dq1 = [0 -45 0];     % grados/s
dq2 = [0 1.5 0];     % m/s
dq3 = [0 22.5 0];    % grados/s
dq4 = [0 -1 0];      % m/s

T = 1; % cada tramo dura 1 segundo

% Eslabón 1
[a11,b11,c11,d11] = cubicSplineCoeffs(q1(1), q1(2), dq1(1), dq1(2), T);
[a12,b12,c12,d12] = cubicSplineCoeffs(q1(2), q1(3), dq1(2), dq1(3), T);

% Eslabón 2
[a21,b21,c21,d21] = cubicSplineCoeffs(q2(1), q2(2), dq2(1), dq2(2), T);
[a22,b22,c22,d22] = cubicSplineCoeffs(q2(2), q2(3), dq2(2), dq2(3), T);

% Eslabón 3
[a31,b31,c31,d31] = cubicSplineCoeffs(q3(1), q3(2), dq3(1), dq3(2), T);
[a32,b32,c32,d32] = cubicSplineCoeffs(q3(2), q3(3), dq3(2), dq3(3), T);

% Eslabón 4
[a41,b41,c41,d41] = cubicSplineCoeffs(q4(1), q4(2), dq4(1), dq4(2), T);
[a42,b42,c42,d42] = cubicSplineCoeffs(q4(2), q4(3), dq4(2), dq4(3), T);


% Crear vectores de tiempo
t1 = linspace(0,1,100);
t2 = linspace(1,2,100);

% Coeficientes por eslabón (cada fila: [a1 b1 c1 d1 a2 b2 c2 d2])
coef = [
    a11 b11 c11 d11 a12 b12 c12 d12;
    a21 b21 c21 d21 a22 b22 c22 d22;
    a31 b31 c31 d31 a32 b32 c32 d32;
    a41 b41 c41 d41 a42 b42 c42 d42
];

% Evaluación
figure
for k = 1:4
    [t_all, q_all] = evalSplineTramos(coef(k,1),coef(k,2),coef(k,3),coef(k,4), ...
                                      coef(k,5),coef(k,6),coef(k,7),coef(k,8));
    subplot(2,2,k)
    plot(t_all,q_all,'LineWidth',2)
    xlabel('Tiempo (s)')
    ylabel(['q' num2str(k)])
    title(['Spline cúbico eslabón ' num2str(k)])
    grid on
end

