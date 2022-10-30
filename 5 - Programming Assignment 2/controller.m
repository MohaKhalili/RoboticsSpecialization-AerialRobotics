function [ u1, u2 ] = controller(~, state, des_state, params)
%   CONTROLLER  Controller for the planar quadrotor

%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]

%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]

%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

kv_z = 20;
kp_z = 200;
kv_phi = 100;
kp_phi = 1000;
kv_y = 10;
kp_y = 10;

pos_error = des_state.pos - state.pos;
vel_error = des_state.vel - state.vel;

phi_commanded = -(1/params.gravity) * (des_state.acc(1,1) + kv_y*(vel_error((1))) + kp_y*(pos_error(1)));
phi_error = phi_commanded - state.rot;

phidot_commanded = 0;
phidot_eror = phidot_commanded - state.omega;

phi_double_dot = 0;

u1 = params.mass * (params.gravity + des_state.acc(2,1) + kv_z*(vel_error(2)) + kp_z*(pos_error(2)));
u2 = params.Ixx * (phi_double_dot + kv_phi*(phidot_eror) + kp_phi*(phi_error));

%   FILL IN YOUR CODE HERE

end

