%% Variables simbólicas
syms q1 q2 q3 q4 L2 L3 real
pi2 = sym(pi)/2;

%% Matrices DH
A1 = [ 1  0  0  0;
       0  1  0  0;
       0  0  1  q1;
       0  0  0  1];

A2 = [ cos(q2+pi2)  -sin(q2+pi2)  0  L2*cos(q2+pi2);
       sin(q2+pi2)   cos(q2+pi2)  0  L2*sin(q2+pi2);
       0             0            1  0;
       0             0            0  1];

A3 = [ cos(q3-pi2)  0   -sin(q3-pi2)  L3*cos(q3-pi2);
       sin(q3-pi2)  0   cos(q3-pi2)  L3*sin(q3-pi2);
       0            -1              0  0;
       0            0              0  1];

A4 = [ 1  0  0  0;
       0  1  0  0;
       0  0  1  q4;
       0  0  0  1];

%% Transformación total
T = simplify(A1*A2*A3*A4)

%% Vector de posición del extremo
p = simplify(T(1:3,4))

%% Jacobiana analítica lineal (3x4)
Jv = simplify(jacobian(p, [q1 q2 q3 q4]))

%% Ejes z acumulados (para jacobiana geométrica angular)
A01 = A1;
A02 = simplify(A1*A2)
A03 = simplify(A1*A2*A3)

z0 = [0;0;1]        % eje z0 en base
z1 = A01(1:3,3)     % eje z1 en base
z2 = A02(1:3,3)     % eje z2 en base
z3 = A03(1:3,3)     % eje z3 en base

%% Jacobiana geométrica angular (3x4)
% Tipo de juntas: q1=P, q2=R, q3=R, q4=P
Jw = [ [0;0;0], z1, z2, [0;0;0] ]
