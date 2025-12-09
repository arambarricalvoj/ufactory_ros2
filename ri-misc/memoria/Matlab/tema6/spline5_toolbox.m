clear all; close all; clc;

q0 = [90 1 0 4];   % posiciones iniciales
qf = [0 4 45 2];   % posiciones finales
N = 50;            % número de muestras
[Q, Qd, Qdd] = jtraj(q0, qf, N);

% Vector de tiempo normalizado
t = linspace(0,1,N);

% Graficar posiciones
figure;
plot(t, Q, 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Posición articular');
legend('q1','q2','q3','q4');
title('Trayectoria quíntica - posiciones');
grid on;

% Graficar velocidades
figure;
plot(t, Qd, 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Velocidad articular');
legend('q1','q2','q3','q4');
title('Trayectoria quíntica - velocidades');
grid on;

% Graficar aceleraciones
figure;
plot(t, Qdd, 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Aceleración articular');
legend('q1','q2','q3','q4');
title('Trayectoria quíntica - aceleraciones');
grid on;