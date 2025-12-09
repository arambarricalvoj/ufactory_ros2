syms q1 q2 q3 q4 L1 L3 real
pi2 = sym(pi)/2;

% A1: theta=q1, d=0, a=L1, alpha=0
A1 = [ cos(q1) -sin(q1) 0 L1*cos(q1);
       sin(q1)  cos(q1) 0 L1*sin(q1);
       0        0       1 0;
       0        0       0 1];

% A2: theta=0, d=q2, a=0, alpha=-pi/2
A2 = [ 1  0  0  0;
       0  0  1  0;
       0 -1  0  q2;
       0  0  0  1];

% A3: theta=q3, d=0, a=0, alpha=+pi/2
A3 = [ cos(q3)  0  sin(q3)  0;
       sin(q3)  0 -cos(q3)  0;
       0        1  0        0;
       0        0  0        1];

% A4: theta=0, d=L3+q4, a=0, alpha=0
A4 = [ 1 0 0 0;
       0 1 0 0;
       0 0 1 L3+q4;
       0 0 0 1];

% Transformación total
T02 = simplify(A1*A2)
T = simplify(A1*A2*A3*A4)

% Vector de posición del extremo
p = T(1:3,4);

% Jacobiana analítica lineal (3x4)
Jv = simplify(jacobian(p, [q1 q2 q3 q4]))


%% Cinemática hasta el segundo eslabón mediante cuaterniones
% Vectores de traslación construidos a partir de la tabla D-H
p1 = [L1; 0; 0];
p2 = [0; 0; q2];

% Cuaterniones de rotación construidos a partir de la tabla D-H
Q1 = [cos(q1/2), 0, 0, sin(q1/2)];                 % Rotación alrededor de Z
Q2 = [cos(-pi/4), sin(-pi/4), 0, 0];               % Rotación alrededor de X (-90°)
R2 = [1, 0, 0, 0];                                 % Identidad

%% Funciones auxiliares
quatConj = @(Q) [Q(1), -Q(2), -Q(3), -Q(4)];

quatMult = @(Q,P) [ ...
    Q(1)*P(1) - (Q(2)*P(2) + Q(3)*P(3) + Q(4)*P(4)), ...
    Q(1)*P(2) + Q(2)*P(1) + Q(3)*P(4) - Q(4)*P(3), ...
    Q(1)*P(3) + Q(3)*P(1) + Q(4)*P(2) - Q(2)*P(4), ...
    Q(1)*P(4) + Q(4)*P(1) + Q(2)*P(3) - Q(3)*P(2) ];

% Rotación de un vector mediante conjugación
quatRotate = @(Q,v) quatMult(quatMult(Q,[0 v(:)']), quatConj(Q));

%% Propagación de posición

% Segundo eslabón: primero se traslada y luego rota
a2 = [0;0;0]; % extremo en el origen de S2
a1_quat = quatMult(quatMult(Q2,[0 a2']), quatConj(Q2)); % rotación de a2 (nulo)
a1_vec = a1_quat(2:4).' + p2; % suma traslación p2

% Primer eslabón: rota y luego traslada
a0_quat = quatMult(quatMult(Q1,[0 (a1_vec + p1)']), quatConj(Q1));
a0 = simplify(a0_quat(2:4).');

disp('Posición del extremo respecto a la base:')
disp(a0)

disp('Comparación del resultado mediante MTH y cuaterniones (diferencia):')
simplify(T02(1:3,4) - a0) % todo 0 indica que son iguales

%% Propagación de orientación
R0 = quatMult(Q1, quatMult(Q2, R2));
disp('Orientación del extremo respecto a la base (cuaternión):')
disp(simplify(R0))

%% Propagación de orientación
R0 = quatMult(Q1, quatMult(Q2, R2));
disp('Orientación del extremo respecto a la base (cuaternión):')
simplify(R0)

