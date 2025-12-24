clear; clc;

% Definir símbolos
syms q1 q2 q3 q4 q5 q6 a2 T2_offset T3_offset real
pi2 = sym(pi)/2;

% Definición de eslabones según la tabla DH
L1_link = Revolute('d', 267, 'a', 0, 'alpha', -pi2);              % Eslabón 1
L2_link = Revolute('a', a2, 'alpha', 0, 'offset', T2_offset);     % Eslabón 2
L3_link = Revolute('a', 77.5, 'alpha', -pi2, 'offset', T3_offset);% Eslabón 3
L4_link = Revolute('d', 342.5, 'alpha', pi2);                     % Eslabón 4
L5_link = Revolute('a', 76, 'alpha', -pi2);                       % Eslabón 5
L6_link = Revolute('d', 97, 'alpha', 0);                          % Eslabón 6

% Crear el robot
robot = SerialLink([L1_link L2_link L3_link L4_link L5_link L6_link], 'name', 'MiRobot');

% Vector de articulaciones
q = [q1 q2 q3 q4 q5 q6];

% Construir la transformación total manualmente con las matrices .T
T01 = simplify(L1_link.A(q1).T);
T12 = simplify(L2_link.A(q2).T);
T23 = simplify(L3_link.A(q3).T);
T34 = simplify(L4_link.A(q4).T);
T45 = simplify(L5_link.A(q5).T);
T56 = simplify(L6_link.A(q6).T);

% Transformación total simbólica
T06_sym = simplify(T01 * T12 * T23 * T34 * T45 * T56);

disp('Transformación total T_0^6 simbólica:');
disp(T06_sym);

% Sustitución numérica
S.q1 = 0; S.q2 = 0; S.q3 = 0; S.q4 = 0; S.q5 = 0; S.q6 = 0;
S.a2 = 289.48866;
S.T2_offset = deg2rad(-79.34995);
S.T3_offset = deg2rad(79.34995);

T06_num = subs(T06_sym, S);
T06_num = vpa(T06_num, 10);

disp('Transformación total T_0^6 numérica:');
disp(T06_num);
