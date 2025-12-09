clear; clc;

syms q1 q2 q3 q4 q1d q2d q3d q4d L1 L3 real;
q = [q1 q2 q3 q4].';
qd = [q1d q2d q3d q4d].';
pi2 = sym(pi)/2;

% DH estándar: [theta d a alpha], prismática: 'sigma',1
L1_link = Revolute('a', L1); % theta1 variable (rotacional)
L2_link = Prismatic('alpha', -pi2); % d2 variable (prismática)
L3_link = Revolute('alpha', pi2);% theta3 variable (rotacional)
L4_link = Prismatic('offset', L3);% d4 variable (prismática) + L3
robot = SerialLink([L1_link, L2_link, L3_link, L4_link], 'name', 'Ex9');

% Cinemática directa y posición del efector
T = robot.fkine(q)
p = transl(T);

% Jacobiana geométrica
J = robot.jacob0(q); % 6x4, marco base
Jv = simplify(J(1:3, :)); % Bloque lineal
Jw = simplify(J(4:6, :)); % Bloque angular

% Velocidad lineal respecto de la base
v = simplify(Jv * qd);

% Velocidad angular respecto de la base
omega0 = simplify(Jw * qd);           

% Jacobiana geométrica del extremo respecto de su marco
Jn = robot.jacobe(q);  
Jvn = simplify(Jn(1:3, :));
Jwn = simplify(Jn(4:6,:));
vn = simplify(Jv * qd);
omegan = simplify(Jwn * qd);    

% Obtener las matrices intermedias
% Matriz de cada eslabón respecto al anterior
T01 = L1_link.A(q(1)).T;   % T_1^0
T12 = L2_link.A(q(2)).T;   % T_2^1
T23 = L3_link.A(q(3)).T;   % T_2^1
T34 = L4_link.A(q(4)).T;   % T_2^1

T02 = simplify(T01 * T12);    % T_2^0
T03 = simplify(T01 * T12 * T23);    % T_3^0
T04 = simplify(T01 * T12 * T23 * T34);    % T_4^0

disp('T_2^0 ='); disp(T02);
disp('T_3^0 ='); disp(T03);
disp('T_4^0 ='); disp(T04);

