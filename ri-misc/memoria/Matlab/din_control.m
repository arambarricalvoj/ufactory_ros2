clear all; close all; clc;

% Offsets articulares (en radianes)
T2_offset_val = deg2rad(-79.34995);
T3_offset_val = deg2rad(79.34995);

% Parámetro geométrico
a2_val = 0.28948866; % en metros

% Definición de eslabones según la tabla DH modificada, en SI
L1 = Link([0    0.267     0        0        0  0],           'modified'); % Eslabón 1
L2 = Link([0    0         0       -pi/2     0  T2_offset_val],'modified');% Eslabón 2
L3 = Link([0    0         a2_val   0        0  T3_offset_val],'modified');% Eslabón 3
L4 = Link([0    0.3425    0.0775  -pi/2     0  0],           'modified'); % Eslabón 4
L5 = Link([0    0         0        +pi/2     0  0],          'modified'); % Eslabón 5
L6 = Link([0    0.097     0.076   -pi/2     0  0],           'modified'); % Eslabón 6

% Crear el robot
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'xArm6_DHmod');

% Visualización del robot con q_i=0
% q = [0 0 0 0 0 0];
% robot.plot(q);

% Parámetros dinámicos
robot.links(1).m = 2.177;
robot.links(1).r = [0.00015 0.02724 -0.01357];
robot.links(1).I = [0.005433 0.004684 0.0031118 9.864e-06 -2.68e-05 -0.000826936];

robot.links(2).m = 2.011;
robot.links(2).r = [0.0367 -0.22088 0.03356];
robot.links(2).I = [0.0271342 0.0053854 0.0262093 0.004736 0.00068673 -0.0047834];

robot.links(3).m = 1.725;
robot.links(3).r = [0.06977 0.1135 0.01163];
robot.links(3).I = [0.006085 0.0036652 0.0057045 -0.0015 0.0009558 0.0018091];

robot.links(4).m = 1.211;
robot.links(4).r = [-0.0002 0.02 -0.026];
robot.links(4).I = [0.0046981 0.0042541 0.00123664 -6.486e-06 -1.404e-05 -0.0002877];

robot.links(5).m = 1.206;
robot.links(5).r = [0.06387 0.02928 0.0035];
robot.links(5).I = [0.0013483 0.00175694 0.002207 -0.00042677 0.00028758 0.0001244];

robot.links(6).m = 0.170;
robot.links(6).r = [0 -0.00677 -0.01098];
robot.links(6).I = [9.3e-05 5.87e-05 0.000132 0 0 -3.6e-06];

for i = 1:6
    robot.links(i).G  = 1;
    robot.links(i).Jm = 0;
    robot.links(i).B  = 0;
    robot.links(i).Tc = [0 0];
end

% Gravedad en SI
grav = [0; 0; -9.81];

% Configuración articular (ejemplo)
q   = [pi/4 pi/4 pi/4 pi/4 pi/4 pi/4];

% Velocidades aleatorias dentro de ±2 rad/s
qd  = [ 1.2, -0.8,  0.5, -1.0,  0.7, -0.6];  % rad/s

% Aceleraciones aleatorias dentro de ±10 rad/s²
qdd = [ 5.0, -3.5,  2.0, -4.0,  6.0, -2.5]; % rad/s^2

% Cálculo de pares dinámicos
tau = robot.rne(q, qd, qdd, grav);
disp('Par calculado en cada articulación [Nm]:');
disp(tau);

