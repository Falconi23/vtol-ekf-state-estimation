% From ekf_copy.m, the EKF generator of the Sapienza Flight Team autopilot (MATLAB + CasADi).
% Omitted: CasADi path setup and param.mat loading. DT, R_GNSS, R_BARO and
% SIGMA_* are placeholders; the tuned flight values are not published.

import casadi.*;

param.dt     = DT;          % [s]
param.r_gnss = R_GNSS;      % GNSS antenna lever arm, body frame [m] (3x1)
param.r_bar  = R_BARO;      % barometer lever arm, body frame [m] (3x1)
param.g      = 9.80665;     % [m/s^2], default if not in param.mat
param.e_3    = [0; 0; 1];   % NED down axis

%% State (nx = 20)
%   p          world NED position                     [m]
%   q          body->world quaternion, scalar last [x y z w]
%   v          world velocity                         [m/s]
%   omega      body angular rate                      [rad/s]
%   a          world acceleration, gravity excluded   [m/s^2]
%   b_p_3_gps  GNSS altitude bias                     [m]
%   b_a_ahrs   accelerometer bias, body frame         [m/s^2]
p         = SX.sym('p', 3);
q         = SX.sym('q', 4);
v         = SX.sym('v', 3);
omega     = SX.sym('omega', 3);
a         = SX.sym('a', 3);
b_p_3_gps = SX.sym('b_p_3_gps', 1);
b_a_ahrs  = SX.sym('b_a_ahrs', 3);

x  = [p; q; v; omega; a; b_p_3_gps; b_a_ahrs];
nx = size(x, 1);

%% Measurements (ny = 17), built in 03_measurement_model.m
%   p_gnss (3), h_bar (1), v_gnss (3), a_ahrs (3), q_ahrs (4), omega_ahrs (3)
ny = 17;

%% Noise covariances (diagonal, tuned per block)
% Small p/q/v entries make the filter trust the kinematic model. omega and a
% are random walks, so their entries set how fast they may change. The
% biases are nearly constant.
param.V = zeros(nx, nx);
param.V(1:3, 1:3)     = eye(3) * SIGMA_P;
param.V(4:7, 4:7)     = eye(4) * SIGMA_Q;
param.V(8:10, 8:10)   = eye(3) * SIGMA_V;
param.V(11:13, 11:13) = eye(3) * SIGMA_OMEGA;
param.V(14:16, 14:16) = eye(3) * SIGMA_A;
param.V(17, 17)       = SIGMA_B_GPS;
param.V(18:20, 18:20) = eye(3) * SIGMA_B_ACC;

% Conservative on GNSS velocity and especially the accelerometer, to reject
% airframe vibration.
param.W = zeros(ny, ny);
param.W(1:3, 1:3)     = eye(3) * SIGMA_GNSS_P;
param.W(4, 4)         = SIGMA_BARO;
param.W(5:7, 5:7)     = eye(3) * SIGMA_GNSS_V;
param.W(8:10, 8:10)   = eye(3) * SIGMA_ACC;
param.W(11:14, 11:14) = eye(4) * SIGMA_AHRS_Q;
param.W(15:17, 15:17) = eye(3) * SIGMA_GYRO;
