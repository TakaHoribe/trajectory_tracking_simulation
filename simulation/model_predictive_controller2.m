function [u, debug_info] = model_predictive_controller2(state, t, ref, param)
% 
% state = [x, y, yaw, delta]
% u = [v_des, delta_des]
% ref = [x_ref; y_ref; yaw_ref; v_ref; k_ref; t_ref];
% 
% 
% 
deg2rad = pi / 180;

IDX_X = 1;
IDX_Y = 2;
IDX_XY = 1:2;
IDX_YAW = 3;
IDX_VEL = 4;
IDX_CURVATURE = 5;
IDX_TIME = 6;

IDX_DELTA = 4;

%% calculate nearest point

% calculate nearest point (use as initial state)
distance = vecnorm(ref(:, IDX_XY)' - state(IDX_XY)');
[~, min_index] = min(distance);
ref_sp = ref(min_index, :);
v_ref = ref_sp(IDX_VEL);

% (debug) calculate latitude error
sp_yaw = ref_sp(IDX_YAW);
T_xy2lonlat = [cos(sp_yaw), sin(sp_yaw);
             -sin(sp_yaw), cos(sp_yaw)];
error_xy = (state(IDX_XY) - ref_sp(IDX_XY))';
error_lonlat = T_xy2lonlat * error_xy;
error_lat = error_lonlat(2);


%% update error dynamics for holizon

% set mpc parameters
mpc_dt = param.mpc2_dt;
mpc_n = param.mpc2_n;
Q = param.mpc2_Q;
R = param.mpc2_R;
t_ = ref_sp(IDX_TIME);
DIM_STATE = 4;
DIM_OUTPUT = 3;
DIM_INPUT = 1;
A_ext = zeros(DIM_STATE*mpc_n, DIM_STATE);
B_ext = zeros(DIM_STATE*mpc_n, DIM_INPUT*mpc_n);
W_ext = zeros(DIM_STATE*mpc_n, 1);
C_ext = zeros(DIM_OUTPUT*mpc_n, DIM_STATE*mpc_n);
Q_ext = zeros(DIM_OUTPUT*mpc_n, DIM_OUTPUT*mpc_n);
R_ext = zeros(DIM_INPUT*mpc_n, DIM_INPUT*mpc_n);
mpc_ref_v = zeros(length(mpc_n), 1);
ref_vec = zeros(mpc_n,5);


for i = 1
    ref_i_ = interp1q(ref(:, IDX_TIME), ref(:,1:5), t_);
    ref_vec(i,:) = ref_i_;
    v_ = ref_i_(IDX_VEL);
    [Ad, Bd, wd, Cd] = get_linearized_state_matrix(mpc_dt, ref_i_, param.wheelbase, param.tau);
    
    cols_i = (i-1)*DIM_STATE+1:i*DIM_STATE;
    A_ext(cols_i, :) = Ad;
    
    rows_i = (i-1)*DIM_INPUT+1:i*DIM_INPUT;
    B_ext(cols_i, rows_i) = Bd;
    
    W_ext(cols_i) = wd;
    
    cols_i_C = (i-1)*DIM_OUTPUT+1:i*DIM_OUTPUT;
    rows_i_C = (i-1)*DIM_STATE+1:i*DIM_STATE;
    C_ext(cols_i_C, rows_i_C) = Cd;
    
    cols_i_Q = (i-1)*DIM_OUTPUT+1:i*DIM_OUTPUT;
    rows_i_Q = (i-1)*DIM_OUTPUT+1:i*DIM_OUTPUT;
    Q_ext(cols_i_Q, rows_i_Q) = Q;
    
    R_ext(rows_i, rows_i) = R;
    
    mpc_ref_v(i) = v_;
    
    t_ = t_ + mpc_dt;
    if t_ > ref(end, IDX_TIME)
        t_ = ref(end, IDX_TIME);
        disp('[MPC] path is too short to predict dynamics');
    end
end

for i = 2:mpc_n
    ref_i_ = interp1q(ref(:, IDX_TIME), ref(:,1:5), t_);
    ref_vec(i,:) = ref_i_;
    v_ = ref_i_(IDX_VEL);
    [Ad, Bd, wd, Cd] = get_linearized_state_matrix(mpc_dt, ref_i_, param.wheelbase, param.tau);
    
    
    cols_i = (i-1)*DIM_STATE+1:i*DIM_STATE;
    cols_i_prev = (i-2)*DIM_STATE+1:(i-1)*DIM_STATE;
    A_ext(cols_i, :) = Ad * A_ext(cols_i_prev, :);
    
    for j = 1:i-1
        rows_j = (j-1)*DIM_INPUT+1:j*DIM_INPUT;
        B_ext(cols_i, rows_j) = Ad * B_ext(cols_i_prev, rows_j);
    end
    rows_i = (i-1)*DIM_INPUT+1:i*DIM_INPUT;
    B_ext(cols_i, rows_i) = Bd;
    
    W_ext(cols_i) = Ad * W_ext(cols_i_prev) + wd;
    
    cols_i_C = (i-1)*DIM_OUTPUT+1:i*DIM_OUTPUT;
    rows_i_C = (i-1)*DIM_STATE+1:i*DIM_STATE;
    C_ext(cols_i_C, rows_i_C) = Cd;
    
    cols_i_Q = (i-1)*DIM_OUTPUT+1:i*DIM_OUTPUT;
    rows_i_Q = (i-1)*DIM_OUTPUT+1:i*DIM_OUTPUT;
    Q_ext(cols_i_Q, rows_i_Q) = Q;
    
    R_ext(rows_i, rows_i) = R;
    
    mpc_ref_v(i) = v_;
    
    t_ = t_ + mpc_dt;
    if t_ > ref(end, IDX_TIME)
        t_ = ref(end, IDX_TIME);
        disp('[MPC] path is too short to predict dynamics');
    end
end


%% convex optimization

x0 = state';

% for yaw singularity
% for i=:length(ref_vec(:,1))
%     if ()
    


Y_ref = reshape(transpose(ref_vec(:,1:3)), [], 1);

% The problem is to solve following for U.
%   1/2 * U'* mat1 * U + mat2 * U + C = 0
mat1 = B_ext' * C_ext' * Q_ext * C_ext * B_ext + R_ext;
mat2 = (C_ext * A_ext * x0 + C_ext * W_ext - Y_ref)' * Q_ext * C_ext * B_ext;


% --- convex optimization ---
%   minimize for x, s.t.
%   J(x) = 1/2 * x' * H * x + f' * x, 
%   A*x <= b,   lb <= x <= ub

H_ = (mat1 + mat1') / 2;
f_ = mat2;
A_ = [];
b_ = [];

% constraint for upper and lower steering boundary
lb_ = -param.mpc2_steering_lim_deg * deg2rad * ones(mpc_n * DIM_INPUT,1);
ub_ = param.mpc2_steering_lim_deg * deg2rad * ones(mpc_n * DIM_INPUT,1);
options_ = optimoptions('quadprog', 'Algorithm','interior-point-convex', 'Display', 'off');
[x_, fval, exitflag, output, lambda] = quadprog(H_, f_, A_, b_, [], [], lb_, ub_, [], options_);
input_vec = x_;


% for debug: compare with / without constraint optimization
% input_vec_LS = -mat1 \ mat2';
% figure(101);
% plot(input_vec_LS,'bo-'); hold on;
% plot(input_vec,'r*-'); grid on; hold off;


v_des = v_ref;
delta_des = input_vec(1);
u = [v_des, delta_des];

%% (debug) calculate predicted trajectory 

x_ = state;
predictd_states = zeros(length(input_vec), length(state));
for i = 1:length(input_vec)
    x_next = calc_kinematics_model(x_, input_vec(i), mpc_dt, mpc_ref_v(i), param.wheelbase, param.tau);
    predictd_states(i,:) = x_next';
    x_ = x_next;
end

predictd_states_real = reshape(predictd_states', [], 1);

predicted_states_linearized = A_ext*x0 + B_ext*input_vec + W_ext;

debug_info = [input_vec', predictd_states_real', predicted_states_linearized', error_lat];


% for debug 
% predictd_states_real_mat = reshape(predictd_states_real, [], 4);
% predicted_states_linearized_mat = transpose(reshape(predicted_states_linearized, 4, []));
% reshape(predictd_states_real, [], 4);
% figure(1);
% plot(predictd_states_real_mat(:,1), predictd_states_real_mat(:,2),'b*-'); hold on;
% plot(predicted_states_linearized_mat(:,1), predicted_states_linearized_mat(:,2), 'ro-'); hold off;
% xlabel('x [m]'); ylabel('y [m]');
% figure(3);
% plot(input_vec); hold on;
% plot(predicted_error(:,3));hold on;
% plot(predictd_states(:,4)); hold off;


end

%% time varying error dynamics model
% used for error dynamics update
function [Ad, Bd, wd, Cd] = get_linearized_state_matrix(dt, ref, L, tau)
    %
    % x = [x, y, yaw, delta]';
    % u = [delta_com];
    %
    IDX_VEL = 4;
    IDX_YAW = 3;
    IDX_CURVATURE = 5;
    
    v_r = ref(IDX_VEL);
    yaw_r = ref(IDX_YAW);
    delta_r = atan(L*ref(IDX_CURVATURE));
    cos_delta_r_squared_inv = 1 / ((cos(delta_r))^2);

    % difinition for continuous model
    A = [0, 0, -v_r * sin(yaw_r), 0;
         0, 0, v_r*cos(yaw_r),    0;
         0, 0, 0,                v_r/L*cos_delta_r_squared_inv;
         0, 0, 0,                -1/tau];
    B = [0; 0; 0; 1/tau];
    C = [1, 0, 0, 0;
         0, 1, 0, 0;
         0, 0, 1, 0];
    w = [v_r*cos(yaw_r) + v_r*sin(yaw_r)*yaw_r;
         v_r*sin(yaw_r) - v_r*cos(yaw_r)*yaw_r;
         v_r/L*(tan(delta_r)-delta_r*cos_delta_r_squared_inv);
         0];
         
    
    % discretization
    Ad = eye(4) + A * dt;
    Bd = B * dt;
    Cd = C;
    wd = w * dt;
end

%% xy-yaw dynamics model
% used for debug (predicted trajectory calculation)
function x_next = calc_kinematics_model(x, u, dt, v, L, tau)

    % x = [x, y, yaw, delta]
    x_next = zeros(4,1);
    yaw = x(3);
    delta = x(4);
    
    % x
    x_next(1) = x(1) + v*cos(yaw)*dt;
    
    % y
    x_next(2) = x(2) + v*sin(yaw)*dt;
    
    % yaw
    x_next(3) = x(3) + v*tan(delta)/L*dt;
    
    % delta
    x_next(4) = x(4) - (x(4) - u)/tau*dt;

end