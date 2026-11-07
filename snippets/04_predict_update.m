% From ekf_copy.m (Sapienza Flight Team autopilot), continues 03.
% One full EKF step as a symbolic expression, generated as C in 05.

x_hat  = SX.sym('x_hat', nx);
P      = SX.sym('P', nx, nx);
y_meas = SX.sym('y', ny);

[x_hat_pred, y_pred, P_pred] = pred(x_hat, P, F, J_F, h, param.V);
[x_hat_next, P_next] = corr(x_hat_pred, y_meas, y_pred, J_h, P_pred, param.W);

function [x_hat_pred, y_pred, P_pred] = pred(x_hat, P, F, J_F, h, V)
    % A is evaluated at the previous estimate.
    A = J_F(x_hat);
    x_hat_pred = F(x_hat);
    y_pred = h(x_hat_pred);
    P_pred = A * P * A.' + V;
end

function [x_hat_next, P_next] = corr(x_hat_pred, y, y_pred, J_h, P_pred, W)
    import casadi.*;
    C = J_h(x_hat_pred);

    % Measurement mask. Single-rate filter, so all 17 are fused and T = I.
    T = SX.eye(size(y, 1));
    r_val = T * (y - y_pred);
    C_val = T * C;
    W_val = T * W * T;

    % 1e-8 keeps S invertible in the generated C in degenerate cases.
    S = C_val * P_pred * C_val.' + W_val + SX.eye(size(y, 1)) * 1e-8;

    K = (P_pred * C_val.') / S;

    x_hat_next = x_hat_pred + K * r_val;

    % The additive update leaves the unit sphere.
    x_hat_next(4:7) = x_hat_next(4:7) / norm(x_hat_next(4:7));

    % Joseph form: stays symmetric PSD under round-off or a suboptimal gain.
    I_KC = SX.eye(size(K, 1)) - K * C_val;
    P_next = I_KC * P_pred * I_KC.' + K * W_val * K.';

    P_next = 0.5 * (P_next + P_next.');
end
