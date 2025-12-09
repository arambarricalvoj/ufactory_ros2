% --- Variables simbólicas ---
syms q1 q2 q3 q1d q2d q3d L1 L2 L3 real
q  = [q1 q2 q3]; % Articulaciones
qd = [q1d; q2d; q3d]; % Velocidades de las articulaciones

% --- Definición de eslabones DH estándar ---
% Formato: Link([theta d a alpha])
% Como son rotacionales, theta=0 en la definición y se sustituye con q al evaluar
% Usamos sym(pi)/2 para que no aparezcan fracciones enormes
pi2 = sym(pi)/2;

L(1) = Link([0   0   0    pi2]);    % theta1 = q1
L(2) = Link([0   L1  0   -pi2]);    % theta2 = q2
L(3) = Link([0   L2  L3   0   ]);   % theta3 = q3

% --- Crear el robot ---
robot = SerialLink(L, 'name', 'manip3R');

% --- Cinemática directa ---
T = robot.fkine(q);        % transformación homogénea simbólica
p = transl(T);             % posición simbólica del extremo

%disp('Posición simbólica del extremo p(q):');
%pretty(simplify(p));

% --- Jacobiana geométrica ---
J = robot.jacob0(q);       % Jacobiana en el marco base
Jv = simplify(J(1:3,:));   % bloque lineal
Jw = simplify(J(4:6,:));   % bloque angular

%disp('Jacobiana geométrica J(q):');
%pretty(J);

%disp('Bloque lineal Jv(q):');
%pretty(Jv);

%disp('Bloque angular Jw(q):');
%pretty(Jw);

% --- Velocidades del efector ---
v = simplify(J * qd);      % [v; omega] simbólico
v_lin = v(1:3);
v_ang = v(4:6);

%disp('Velocidad lineal simbólica v(q, qdot):');
%pretty(v_lin);

%disp('Velocidad angular simbólica w(q, qdot):');
%pretty(v_ang);


% --- Comparación automática de Jacobianas (Toolbox vs calculada manual) ---

% --- Definir posición del extremo (calculado manualmente)  ---
x = L3*(cos(q1)*cos(q2)*cos(q3) - sin(q1)*sin(q3)) - L2*cos(q1)*sin(q2) + L1*sin(q1);
y = L3*(sin(q1)*cos(q2)*cos(q3) + cos(q1)*sin(q3)) - L2*sin(q1)*sin(q2) - L1*cos(q1);
z = L3*sin(q2)*cos(q3) + L2*cos(q2);

% --- Jacobiana manual ---
J_manual = [ ...
   -y, -cos(q1)*z, -L3*(sin(q1)*cos(q3) + cos(q1)*cos(q2)*sin(q3));
    x, -sin(q1)*z,  L3*(cos(q1)*cos(q3) - sin(q1)*cos(q2)*sin(q3));
    0,  L3*cos(q2)*cos(q3) - L2*sin(q2), -L3*sin(q2)*sin(q3);
    0,  sin(q1), -cos(q1)*sin(q2);
    0, -cos(q1), -sin(q1)*sin(q2);
    1,  0,        cos(q2) ];

% --- Comparación ---
Diff = simplify(J - J_manual);

disp('Diferencia J_toolbox - J_manual (simplificada):');
pretty(Diff)
