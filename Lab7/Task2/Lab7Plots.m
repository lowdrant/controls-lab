clear; clc; close all
daty = xlsread('YData.xlsx');
datu = xlsread('UData.xlsx');

figure(), subplot(121), title 'Y Data', hold on
plot([0:size(daty, 1)-1].*0.001, daty(:, 1))
plot([0:size(daty, 1)-1].*0.001, daty(:, 2))
legend('ref', 'data')
xlabel 'Time (s)', ylabel('$\theta$ (rad)', 'interpreter', 'latex') 

subplot(122), title 'U data', hold on
plot([0:size(datu, 1)-1].*0.001, datu(:, 1))
plot([0:size(datu, 1)-1].*0.001, datu(:, 2))
legend('ref', 'data'), ylabel 'Voltage (V)', xlabel 'Time (s)'