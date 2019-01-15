function [u, debug_info] = pure_pursuit(state, t, ref, param)
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
% IDX_CURVATURE = 5;
% IDX_TIME = 6;


lookahead_dist = param.pure_pursuit_lookahead;

distance = vecnorm(ref(:,IDX_XY)' - state(IDX_XY)');
[~, min_index] = min(distance);
pr = ref(min_index, :);

v_des = pr(IDX_VEL);

for i = min_index:length(ref)
    dist = norm(ref(i, IDX_XY) - state(:,IDX_XY));
    if dist > lookahead_dist
        break;
    end
end
lookahead_pt = ref(i, :);

alpha = atan2(lookahead_pt(IDX_Y) - state(IDX_Y), lookahead_pt(IDX_X) - state(IDX_X)) - state(IDX_YAW);

omega_des = 2 * v_des * sin(alpha) / param.wheelbase;
delta_des = atan2(omega_des * param.wheelbase, v_des);



u = [v_des, delta_des];


% lattitude error calc for debug
sp_yaw = pr(IDX_YAW);
T_xy2lonlat = [cos(sp_yaw), sin(sp_yaw);
             -sin(sp_yaw), cos(sp_yaw)];
error_xy = (state(IDX_XY) - pr(IDX_XY))';
error_lonlat = T_xy2lonlat * error_xy;
error_lat = error_lonlat(2);

debug_info = [lookahead_pt(IDX_X), lookahead_pt(IDX_Y), lookahead_pt(IDX_YAW), error_lat];