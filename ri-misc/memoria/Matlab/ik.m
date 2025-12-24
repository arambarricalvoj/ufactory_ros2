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
R06 = T06(1:3,1:3);
p06 = T06(1:3,4);

disp('=== ECUACIONES DE POSICIÓN p06 ===');
disp('p_x ='); pretty(p06(1))
disp('p_y ='); pretty(p06(2))
disp('p_z ='); pretty(p06(3))

disp(' ');
disp('=== ECUACIONES DE ORIENTACIÓN R06 ===');
pretty(R06)

