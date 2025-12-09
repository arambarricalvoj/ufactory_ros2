clear all; close all; clc;

qf = UnitQuaternion(cos(pi/4), [0 0 sin(pi/4)]); % cuaternión final
qi = UnitQuaternion();                           % cuaternión identidad (orientación inicial)

% Interpolación en t=0.5 y t=0.7
q05 = qi.interp(qf, 0.5);
q07 = qi.interp(qf, 0.7);

disp('Orientación en t=0.5:')
disp(q05)
theta05 = 2*acos(q05.s)
disp('Orientación en t=0.7:')
disp(q07)
theta07 = 2*acos(q07.s)
