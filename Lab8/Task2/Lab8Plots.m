clear; clc; close all
dat = xlsread('Lab8Data.xlsx');
datu = dat(1:floor(end/2), 1);
daty = dat(1:floor(end/2), 2);

figure(), subplot(211), title 'Y Data', hold on
plot([0:size(daty, 1)-1].*0.001, daty)
xlabel 'Time (s)', ylabel('$\theta$ (rad)', 'interpreter', 'latex') 

subplot(212), title 'U data', hold on
plot([0:size(datu, 1)-1].*0.001, datu)
ylabel 'Voltage (V)', xlabel 'Time (s)'