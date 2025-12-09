clear all; close all; clc;
waypoints = [90 45 0;   % q1 en t=0,1,2
             1  2  4;   % q2
             0 22.5 45; % q3
             4  3  2];  % q4
waypoints = waypoints'; % cada fila = configuración
tpts = [0 1 2];         % instantes donde pasan por los waypoints
tvec = 0:0.05:2;        % vector de tiempo de muestreo

[q, qd, qdd] = quinticpolytraj(waypoints', tpts, tvec);

% graficar posiciones:
figure
plot(tvec, q)
legend('q1','q2','q3','q4')
title('Posición multi-segmento con splines quínticas')

% graficar velocidades:
figure
plot(tvec, qd)
legend('q1','q2','q3','q4')
title('Velocidad multi-segmento con splines quínticas')

% graficar aceleraciones:
figure
plot(tvec, qdd)
legend('q1','q2','q3','q4')
title('Aceleración multi-segmento con splines quínticas')