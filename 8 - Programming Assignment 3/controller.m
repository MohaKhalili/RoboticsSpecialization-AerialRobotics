function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z]
%   state.vel = [x_dot; y_dot; z_dot]
%   state.rot = [phi; theta; psi] 
%   state.omega = [p; q; r ]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z]
%   des_state.vel = [x_dot; y_dot; z_dot]
%   des_state.acc = [x_ddot; y_ddot; z_ddot]
%   des_state.yaw
%   des_state.yawdot
%
%   params: robot parameters:
%   mass
%   I
%   invI
%   gravity
%   arm_length
%   minF
%   maxF

%   Using these current and desired states, you have to compute the desired
%   controls

% =================== Your code goes here ===================
% k_F = 6.11 * 1e-10;
% k_M = 1.50 * 1e-09;
% gamma = k_M / k_F;

% u2_mat = [
%     0 params.arm_length 0 -params.arm_length;
%     -params.arm_length 0 params.arm_length 0;
%     gamma -gamma gamma -gamma
%     ];
% 
% omega_dot = params.invI * u2_mat * [F1;F2;F3;F4];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

K_d = [40;40;20];
K_p = [200;200;100];

r = state.pos;
r_des = des_state.pos;
r_dot = state.vel;
r_dot_des = des_state.vel;
r_dotdot_des = des_state.acc;

r_dotdot_c = r_dotdot_des + K_d .* (r_dot_des - r_dot) + K_p .* (r_des - r);

psi_des = des_state.yaw;
phi_c = (1/params.gravity) * (r_dotdot_c(1)*sin(psi_des) - r_dotdot_c(2)*cos(psi_des));
theta_c = (1/params.gravity) * (r_dotdot_c(1)*cos(psi_des) + r_dotdot_c(2)*sin(psi_des));
psi_c = psi_des;


% Thrust
F = params.mass * (params.gravity + r_dotdot_c(3));

if F<params.minF
    F=params.minF;
end
if F>params.maxF
    F=params.maxF;
end

p = state.omega(1);
q = state.omega(2);
r = state.omega(3);

p_c = 0;
q_c = 0;
r_c = des_state.yawdot;

phi = state.rot(1);
theta = state.rot(2);
psi = state.rot(3);

Kp_phi = 100;
Kd_phi = 2;

Kp_theta = 100;
Kd_theta = 2;

Kp_psi = 100;
Kd_psi = 2;

% Moment
M = [
    Kp_phi * (phi_c - phi) + Kd_phi * (p_c - p);
    Kp_theta * (theta_c - theta) + Kd_theta * (q_c - q);
    Kp_psi * (psi_c - psi) + Kd_psi * (r_c - r)];



% =================== Your code ends here ===================

end
