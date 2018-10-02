%%
% Marion Anderson
% ECE 4550 Fall 2018
% HW 5

clear clc close all

M1 = 1; M2 = 1;

A = [0  0 1 0
     0  0 0 1
    -2  2 0 0
     2 -2 0 0];
B = [0; 0; 1 ;0];
C = [0 1 0 0];

%% 5.1: Controllability

scriptC = B;  % Initial controllability matrix
for n = 1:length(A)-1  % adding to controllability matrix
    scriptC = [scriptC, (A^n) * B];
end

fprintf('\n'); disp('scriptC ='); fprintf('\n')
disp(scriptC)
fprintf('\n'); disp('det(scriptC) ='); fprintf('\n')
disp(det(scriptC))


% The controllability matrix has non-zero determinant, and so must be
% full rank. This system is controllable.

%% 5.2: Regulator Gains

% regulator gain matrix
syms K1 K2 K3 K4
K = [K1 K2 K3 K4];

sR = -10;  % desired pole location

K = acker(A, B, sR*ones(1, length(A)))
eig(A - B*K)

%% 5.3: Observability

scriptO = C;  % Initial observability matrix
for n = 1:length(A)-1  % adding to observability matrix
    scriptO = [scriptO; C * (A^n)];
end

fprintf('\n'); disp('scriptO ='); fprintf('\n')
disp(scriptO)
fprintf('\n'); disp('det(scriptO) ='); fprintf('\n')
disp(det(scriptO))


% The observability matrix has non-zero determinant, and so must be
% full rank. This system is observabile.

%% 5.4: Estimator Gains

% estimator gain matrix
syms L1 L2 L3 L4
L = [L1; L2; L3; L4];

sL = -10;  % desired pole location
AL = A - L*C  % estimator system matrix

% solving for gains
L = acker(A', C', sL*ones(1, length(A)))'
eig(A - L*C)
