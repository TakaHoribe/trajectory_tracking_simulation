function d_state = kinematics_model(state, input, param)

v_des = input(1);
delta_des = input(2);

% limit
delta_des = max(min(delta_des, param.steer_lim), -param.steer_lim);
v_des = max(min(v_des, param.vel_max), param.vel_min);

% x = state(1);
% y = state(2);
yaw = state(3);
delta = state(4);

v = v_des;

d_x = v * cos(yaw);
d_y = v * sin(yaw);
d_yaw = v * tan(delta) / param.wheelbase;
d_delta = - (delta - delta_des) / param.tau;

% add steady state error caused by friction
if abs(delta - delta_des) < param.steering_steady_state_error_deg / 180 * pi
    d_delta = 0;
end


d_state = [d_x, d_y, d_yaw, d_delta];



