clear; clc;

% Punto programado
xb = 3.0;
yb = 3.0;
zb = 3.0;

% Puntos alcanzados
X = [2.9; 2.9; 2.7; 2.6];
Y = [2.8; 2.7; 2.7; 3.0];
Z = [3.0; 3.0; 3.0; 2.9];
n = length(X);

% Baricentro
xm = mean(X);
ym = mean(Y);
zm = mean(Z);

% Precisión (exactitud)
AP = sqrt((xb-xm)^2 + (yb-ym)^2 + (zb-zm)^2);

disp('Baricentro:'); disp([xm, ym, zm]);
disp('Precisión (mm):'); disp(AP);

% Distancias de cada punto al baricentro
Lj = zeros(n,1);
for i=1:n
    Lj(i) = sqrt((X(i)-xm)^2 + (Y(i)-ym)^2 + (Z(i)-zm)^2);
end

% Media y desviación estándar
Lm = mean(Lj);
Ls = std(Lj,1); % desviación estándar muestral (usar n-1)
Ls = std(Lj,0);
RP = Lm + 3*Ls;

disp('Repetibilidad (mm):'); disp(RP);

% --- Gráfica ---
figure; hold on; grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)');

% Puntos alcanzados (negro)
plot(X,Y,'ko','MarkerFaceColor','k');

% Punto programado (azul)
plot(xb,yb,'bs','MarkerFaceColor','b');

% Baricentro (rojo)
plot(xm,ym,'rd','MarkerFaceColor','r');

% Círculo de error medio (verde) y repetibilidad (cyan) en plano XY
theta = linspace(0,2*pi,200);
plot(xm + Lm*cos(theta), ym + Lm*sin(theta),'g-','LineWidth',1.5);
plot(xm + RP*cos(theta), ym + RP*sin(theta),'c-','LineWidth',1.5);

legend('Puntos alcanzados','Punto programado','Baricentro',...
       'Error medio','Repetibilidad','Location','best');
title('Precisión y repetibilidad del robot (proyección XY)');
