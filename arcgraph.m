clear; clc; close all

% --- Datos ---
r  = 77.5;      % radio (mm)
x1 = 0;         % extremo izquierdo (X)
y  = 0;         % posición en el "puente" (Y)
z1 = 0;         % Z de los extremos
ta = 0;         % ángulo inicio (desde la derecha)
tb = 90;        % ángulo fin
ang = 10;       % ángulo a consultar (desde la derecha)

% --- Punto en función del ángulo (derecha a izquierda), [X Y Z] ---
pt = @(t) [x1 + r*(1 + cosd(t)),  y,  z1 + r*sind(t)];

P1 = pt(ta);
P2 = pt((ta + tb)/2);
P3 = pt(tb);
Pa = pt(ang);

% --- Resultados ---
arco = r * deg2rad(tb - ta);
fprintf('\nTramo %d° -> %d°\n', ta, tb);
fprintf('P1 (%3d°): X = %8.2f  Y = %8.2f  Z = %8.2f\n', ta, P1);
fprintf('P2 (%3g°): X = %8.2f  Y = %8.2f  Z = %8.2f\n', (ta+tb)/2, P2);
fprintf('P3 (%3d°): X = %8.2f  Y = %8.2f  Z = %8.2f\n', tb, P3);
fprintf('Arco: %.2f mm | dX: %.2f mm | dZ: %.2f mm\n', arco, P3(1)-P1(1), P3(3)-P1(3));

fprintf('\nA %g° desde la derecha: X = %.2f  Y = %.2f  Z = %.2f\n', ang, Pa);
fprintf('Arco recorrido hasta ahí: %.2f mm\n', r*deg2rad(ang));

% --- Gráfico ---
t_all = linspace(0, 180, 200);
t_rec = linspace(ta, tb, 100);
xz_all = [x1 + r*(1 + cosd(t_all)); z1 + r*sind(t_all)];
xz_rec = [x1 + r*(1 + cosd(t_rec)); z1 + r*sind(t_rec)];

figure; hold on; axis equal; grid on
plot(xz_all(1,:), xz_all(2,:), '--', 'Color', [0.6 0.6 0.6])
plot(xz_rec(1,:), xz_rec(2,:), 'b', 'LineWidth', 3)
plot(P1(1), P1(3), 'o', 'MarkerFaceColor', [0.5 0.5 0.5], 'MarkerSize', 8)
plot(P2(1), P2(3), 'o', 'MarkerFaceColor', 'm', 'MarkerSize', 8)
plot(P3(1), P3(3), 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 8)
plot(Pa(1), Pa(3), 'rx', 'MarkerSize', 12, 'LineWidth', 2)   % ángulo consultado
plot(x1 + r, z1, 'k+')                                        % centro
text(P1(1), P1(3)+5, 'P1'); text(P2(1), P2(3)+5, 'P2'); text(P3(1), P3(3)+5, 'P3');
text(Pa(1), Pa(3)-6, sprintf('%g°', ang), 'Color', 'r');
xlabel('X (mm)'); ylabel('Z (mm)')
title(sprintf('Arco de %d° a %d° (derecha a izquierda)', ta, tb))
