% From ekf_copy.m (Sapienza Flight Team autopilot), continues 02 (uses R, F).
% World frame is NED, so altitude is -z.

% Bias on the vertical channel only: GNSS altitude drifts, x/y do not.
p_gnss = p + R*param.r_gnss + [0; 0; b_p_3_gps];

h_bar = -(p(3) + R(3, :) * param.r_bar);

% Lever-arm velocity: omega is a body rate, so cross in body frame, then rotate.
v_gnss = v + R * cross(omega, param.r_gnss);

% Specific force: gravity is +g*e_3 in NED. AHRS lever-arm terms ignored.
a_ahrs = R.' * (a - param.g*param.e_3) + b_a_ahrs;

q_ahrs     = q;
omega_ahrs = omega;

y  = [p_gnss; h_bar; v_gnss; a_ahrs; q_ahrs; omega_ahrs];
ny = size(y, 1);
h  = Function('h', {x}, {y});

% A is the Jacobian of the full RK4 step including the quaternion
% normalization. Both end up as plain C, no numerical differentiation.
A = jacobian(x_next, x);
C = jacobian(y, x);
J_F = Function('J_F', {x}, {A});
J_h = Function('J_h', {x}, {C});
