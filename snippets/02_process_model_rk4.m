% From ekf_copy.m (Sapienza Flight Team autopilot), continues 01.
% Purely kinematic model: no mass, inertia or thrust, so the filter needs no
% actuator commands. omega and a are random walks pinned by the IMU.

%   p_dot = v, v_dot = a, q_dot = 1/2 Omega(omega) q, the rest constant.
R = local_R_quat(q);   % body -> world, used by the measurement model

p_dot         = v;
v_dot         = a;
q_dot         = (1/2) * local_skew_quat(omega) * q;
omega_dot     = SX.zeros(3, 1);
a_dot         = SX.zeros(3, 1);
b_p_3_gps_dot = SX.zeros(1, 1);
b_a_ahrs_dot  = SX.zeros(3, 1);

x_dot = [p_dot; q_dot; v_dot; omega_dot; a_dot; b_p_3_gps_dot; b_a_ahrs_dot];
f = Function('f', {x}, {x_dot});

% Explicit RK4 rather than the team's low-storage integrator, so CasADi
% differentiates exactly the scheme that runs on board.
k1 = f(x);
k2 = f(x + param.dt/2 * k1);
k3 = f(x + param.dt/2 * k2);
k4 = f(x + param.dt   * k3);
x_next = x + (param.dt/6) * (k1 + 2*k2 + 2*k3 + k4);

% Renormalized inside the symbolic expression, so dF/dx includes it.
x_next(4:7) = x_next(4:7) / norm(x_next(4:7));
F = Function('F', {x}, {x_next});

function R = local_R_quat(q)
    % Body -> world, q = [x y z w].
    xx = q(1)^2; yy = q(2)^2; zz = q(3)^2;
    xy = q(1)*q(2); xz = q(1)*q(3); xw = q(1)*q(4);
    yz = q(2)*q(3); yw = q(2)*q(4); zw = q(3)*q(4);
    R = casadi.SX(3, 3);
    R(1,1) = 1 - 2*(yy+zz); R(1,2) = 2*(xy-zw);     R(1,3) = 2*(xz+yw);
    R(2,1) = 2*(xy+zw);     R(2,2) = 1 - 2*(xx+zz); R(2,3) = 2*(yz-xw);
    R(3,1) = 2*(xz-yw);     R(3,2) = 2*(yz+xw);     R(3,3) = 1 - 2*(xx+yy);
end

function S = local_skew_quat(w)
    % Omega(w) = [-[w]x w; -w' 0] for a scalar-last quaternion.
    S = [ 0,     w(3), -w(2), w(1);
         -w(3),  0,     w(1), w(2);
          w(2), -w(1),  0,    w(3);
         -w(1), -w(2), -w(3), 0   ];
end
