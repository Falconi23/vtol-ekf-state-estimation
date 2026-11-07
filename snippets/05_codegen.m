% From ekf_copy.m (Sapienza Flight Team autopilot), continues 04.
% Omitted: the CasADi CodeGenerator setup (team boilerplate) and ekf_sfun.c,
% the S-Function wrapper written by teammates.

% Input order fixes the Simulink ports: x_hat (20x1), y (17x1), P (20x20).
% The old estimator also took commanded force/torque; the kinematic model
% removed it.
l = Function('l', {x_hat, y_meas, P}, {x_hat_next, P_next});

% [omitted: CodeGenerator('ekf', ...) writes ekf.c / ekf.h from "l" with
%  Simulink types (real_T, int_T) and simstruc.h]

% Rebuilt in the same script so the generated C and the SITL MEX never diverge.
disp('Compiling the S-Function...');
mex('ekf_sfun.c', 'ekf.c');
disp('Code generation and compilation completed.');
