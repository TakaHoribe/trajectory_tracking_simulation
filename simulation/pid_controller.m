function [u, debug_info] = pid_controller(state, t, ref, param)
% 
% state = [x, y, yaw, delta]
% u = [v_des, delta_des]
% ref = [x_ref; y_ref; yaw_ref; v_ref];
% 
% 
% 

IDX_X = 1;
IDX_Y = 2;
IDX_XY = 1:2;
IDX_YAW = 3;
IDX_VEL = 4;
IDX_CURVATURE = 5;
% IDX_TIME = 6;


% LON = 1;
LAT = 2;

kp = 0.3 * 1;
kd = 0.5 * 3;

yaw = state(IDX_YAW);


distance = vecnorm(ref(:,1:2)' - state(1:2)');
[~, min_index] = min(distance);

pr = ref(min_index, :);

v_des = pr(IDX_VEL);


% feedforward input calculation
ff_curvature = atan(param.wheelbase * pr(IDX_CURVATURE));

% coordinate transformation to body frame
T = [cos(yaw), sin(yaw);
    -sin(yaw), cos(yaw)];
error_xy = (state(IDX_XY) - pr(IDX_XY))';
error_latlon = T * error_xy;
error_yaw = yaw - pr(IDX_YAW);

% should use mod?
while (-2*pi <= error_yaw && error_yaw <= 2*pi) == 0 
    if (error_yaw >= 2*pi)
        error_yaw = error_yaw - 2*pi;
    elseif (error_yaw <= -2*pi)
        error_yaw = error_yaw + 2*pi;
    end
end
if (error_yaw > pi)
    error_yaw = error_yaw - 2*pi;
elseif (error_yaw < -pi)
    error_yaw = error_yaw + 2*pi;
end

delta_des = -kp * (error_latlon(LAT)) - kd * error_yaw + ff_curvature;
fb_lat = -kp * (error_latlon(LAT));
fb_yaw = - kd * error_yaw;

u = [v_des, delta_des];

debug_info = [pr(IDX_X), pr(IDX_Y), pr(IDX_YAW), fb_lat, fb_yaw, ff_curvature, error_latlon(LAT)];