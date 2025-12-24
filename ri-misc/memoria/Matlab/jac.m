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

% Jacobiano geométrico (Jv + Jw)
% Jacobiano lineal usando jacobian
Jv = jacobian(p06, [q1 q2 q3 q4 q5 q6]);

% Ejes z_i en el marco base
z0 = [0;0;1];
T01_ = T01;
T02_ = T01*T12;
T03_ = T01*T12*T23;
T04_ = T01*T12*T23*T34;
T05_ = T01*T12*T23*T34*T45;

z1 = T01_(1:3,3);
z2 = T02_(1:3,3);
z3 = T03_(1:3,3);
z4 = T04_(1:3,3);
z5 = T05_(1:3,3);

% Jacobiano angular
Jw = [z0 z1 z2 z3 z4 z5];

% Jacobiano geométrico completo
J = simplify([Jv; Jw]);

disp('=== JACOBIANO GEOMÉTRICO ===');
pretty(J)

% Velocidades simbólicas
syms dq1 dq2 dq3 dq4 dq5 dq6 real
dq = [dq1; dq2; dq3; dq4; dq5; dq6];

% Contribución angular de cada articulación
omega1 = simplify(Jw(:,1) * dq1);
omega2 = simplify(Jw(:,2) * dq2);
omega3 = simplify(Jw(:,3) * dq3);
omega4 = simplify(Jw(:,4) * dq4);
omega5 = simplify(Jw(:,5) * dq5);
omega6 = simplify(Jw(:,6) * dq6);

% Velocidades del efector
vel = simplify(J * dq);
v_linear  = vel(1:3);
v_angular = vel(4:6);

disp('=== VELOCIDAD ANGULAR APORTADA POR CADA ARTICULACIÓN ===');
disp('omega_1 ='); pretty(omega1)
disp(' '); disp('omega_2 ='); pretty(omega2)
disp(' '); disp('omega_3 ='); pretty(omega3)
disp(' '); disp('omega_4 ='); pretty(omega4)
disp(' '); disp('omega_5 ='); pretty(omega5)
disp(' '); disp('omega_6 ='); pretty(omega6)

disp(' ');
disp('=== VELOCIDAD LINEAL DEL EFECTOR ===');
pretty(v_linear)

disp(' ');
disp('=== VELOCIDAD ANGULAR DEL EFECTOR ===');
pretty(v_angular)

% Evaluación numérica de T06, J y velocidades
disp(' ');
disp('===============================================================');
disp('===   EVALUACIÓN NUMÉRICA DE T06, J Y VELOCIDADES          ===');
disp('===============================================================');

% Valores numéricos
S.q1 = 0; S.q2 = 0; S.q3 = 0; S.q4 = 0; S.q5 = 0; S.q6 = 0;
S.a2 = 289.48866;
S.T2_offset = deg2rad(-79.34995);
S.T3_offset = deg2rad(79.34995);

disp(' ');
disp('=== VALORES USADOS PARA LA EVALUACIÓN NUMÉRICA ===');
fprintf('q1 = %.4f rad\n', S.q1);
fprintf('q2 = %.4f rad\n', S.q2);
fprintf('q3 = %.4f rad\n', S.q3);
fprintf('q4 = %.4f rad\n', S.q4);
fprintf('q5 = %.4f rad\n', S.q5);
fprintf('q6 = %.4f rad\n', S.q6);
fprintf('a2 = %.5f mm\n', S.a2);
fprintf('T2_offset = %.4f rad (%.2f°)\n', S.T2_offset, rad2deg(S.T2_offset));
fprintf('T3_offset = %.4f rad (%.2f°)\n', S.T3_offset, rad2deg(S.T3_offset));

% Transformación numérica
T06_num = vpa(subs(T06, S), 10);
disp(' ');
disp('=== TRANSFORMACIÓN HOMOGÉNEA T_0^6 NUMÉRICA ===');
disp(T06_num);

% Jacobiano numérico
J_num = vpa(subs(J, S), 10);
disp(' ');
disp('=== JACOBIANO GEOMÉTRICO NUMÉRICO ===');
disp(J_num);

% Velocidades articulares numéricas
dq_val = [0.2; 0.1; -0.15; 0.05; 0.1; -0.05];
vel_num = double(J_num * dq_val);

v_linear_num  = vel_num(1:3);
v_angular_num = vel_num(4:6);

disp(' ');
disp('=== VELOCIDAD LINEAL DEL EFECTOR (numérica) ===');
disp(v_linear_num.');

disp(' ');
disp('=== VELOCIDAD ANGULAR DEL EFECTOR (numérica) ===');
disp(v_angular_num.');

