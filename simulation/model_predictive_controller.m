function [u, debug_info] = model_predictive_controller(state, t, ref, param)
% 
% state = [x, y, yaw, delta]
% u = [v_des, delta_des]
% ref = [x_ref; y_ref; yaw_ref; v_ref; k_ref; t_ref];
% 
% 
% 

% =================== for delay compensation ==============================
persistent deltades_buffer
if isempty(deltades_buffer)
    deltades_buffer = zeros(param.mpc_delay_comp_step, 1);
end
% =========================================================================
    
deg2rad = pi / 180;
delay_step = param.mpc_delay_comp_step;

IDX_X = 1;
IDX_Y = 2;
IDX_XY = 1:2;
IDX_YAW = 3;
IDX_VEL = 4;
IDX_CURVATURE = 5;
IDX_TIME = 6;

IDX_DELTA = 4;

%% convert xy-yaw to error dynamics

% calculate nearest point (use as initial state)
distance = vecnorm(ref(:, IDX_XY)' - state(IDX_XY)');
[~, min_index] = min(distance);
ref_sp = ref(min_index, :);

% convert x,y to lon,lat model
sp_yaw = ref_sp(IDX_YAW);
T_xy2lonlat = [cos(sp_yaw), sin(sp_yaw);
             -sin(sp_yaw), cos(sp_yaw)];
error_xy = (state(IDX_XY) - ref_sp(IDX_XY))';
error_lonlat = T_xy2lonlat * error_xy;
error_lat = error_lonlat(2);


% calculate yaw error
error_yaw = state(IDX_YAW) - sp_yaw;
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

% initial state for error dynamics
x0 = [error_lat; error_yaw; state(IDX_DELTA)];

%% update error dynamics for holizon

% -- set mpc parameters --
mpc_dt = param.mpc_dt;
mpc_n = param.mpc_n;
Q = param.mpc_Q;
R = param.mpc_R;
mpc_t = ref_sp(IDX_TIME);
DIM_X = 3;
DIM_Y = 2;
DIM_U = 1;
Aex = zeros(DIM_X*mpc_n, DIM_X);
Bex = zeros(DIM_X*mpc_n, DIM_U*mpc_n);
Wex = zeros(DIM_X*mpc_n, 1);
Cex = zeros(DIM_Y*mpc_n, DIM_X*mpc_n);
Qex = zeros(DIM_Y*mpc_n, DIM_Y*mpc_n);
Rex = zeros(DIM_U*mpc_n, DIM_U*mpc_n);
mpc_ref_v = zeros(mpc_n + delay_step, 1);
debug_ref_mat = zeros(mpc_n + delay_step,5);

% =================== for delay compensation ==============================
% -- apply delay compensation : update dynamics with increasing mpt_t --
x_curr = x0;
for i = 1:delay_step
    if mpc_t > ref(end, IDX_TIME)
        mpc_t = ref(end, IDX_TIME);
        disp('[MPC] path is too short to predict dynamics');
    end
    ref_now = interp1q(ref(:, IDX_TIME), ref(:,1:5), mpc_t);
    debug_ref_mat(i,:) = ref_now;
    v_ = ref_now(IDX_VEL);
    k_ = ref_now(IDX_CURVATURE);
    
    % get discrete state matrix
    % NOTE : use control_dt as delta time, not mpc_dt. 
    [Ad, Bd, wd, ~] = get_error_dynamics_state_matrix(param.control_dt, v_, param.wheelbase, param.tau, k_);
    u_now = deltades_buffer(end - i + 1);
    x_next = Ad * x_curr + Bd * u_now + wd;
    
    mpc_t = mpc_t + param.control_dt; % THIS IS NOT mpc_dt, BUT control_dt
    x_curr = x_next;
    
    mpc_ref_v(i) = v_;
end
x0 = x_curr;
% =========================================================================

% -- mpc matrix for i = 1 --
ref_i_ = interp1q(ref(:, IDX_TIME), ref(:,1:5), mpc_t);
debug_ref_mat(1 + delay_step,:) = ref_i_; % MODIFIED FOR DELAY
v_ = ref_i_(IDX_VEL);
k_ = ref_i_(IDX_CURVATURE);
[Ad, Bd, wd, Cd] = get_error_dynamics_state_matrix(mpc_dt, v_, param.wheelbase, param.tau, k_); 
Aex(1:DIM_X, :) = Ad;
Bex(1:DIM_X, 1:DIM_U) = Bd;
Wex(1:DIM_X) = wd;
Cex(1:DIM_Y, 1:DIM_X) = Cd;
Qex(1:DIM_Y, 1:DIM_Y) = Q;
Rex(1:DIM_U, 1:DIM_U) = R;

mpc_ref_v(1 + delay_step) = v_;

% -- mpc matrix for i = 2:n --
for i = 2:mpc_n
    
    % update mpc time
    mpc_t = mpc_t + mpc_dt;
    if mpc_t > ref(end, IDX_TIME)
        mpc_t = ref(end, IDX_TIME);
        disp('[MPC] path is too short to predict dynamics');
    end

    % get reference information
    ref_i_ = interp1q(ref(:, IDX_TIME), ref(:,1:5), mpc_t);
    debug_ref_mat(i + delay_step,:) = ref_i_;
    v_ = ref_i_(IDX_VEL);
    k_ = ref_i_(IDX_CURVATURE);
    
    % get discrete state matrix
    [Ad, Bd, wd, Cd] = get_error_dynamics_state_matrix(mpc_dt, v_, param.wheelbase, param.tau, k_);
    
    % update mpc matrix
    idx_x_i = (i-1)*DIM_X+1:i*DIM_X;
    idx_x_i_prev = (i-2)*DIM_X+1:(i-1)*DIM_X;
    idx_u_i = (i-1)*DIM_U+1:i*DIM_U;
    idx_y_i = (i-1)*DIM_Y+1:i*DIM_Y;
    Aex(idx_x_i, :) = Ad * Aex(idx_x_i_prev, :);
    for j = 1:i-1
        idx_u_j = (j-1)*DIM_U+1:j*DIM_U;
        Bex(idx_x_i, idx_u_j) = Ad * Bex(idx_x_i_prev, idx_u_j);
    end
    Bex(idx_x_i, idx_u_i) = Bd;
    Wex(idx_x_i) = Ad * Wex(idx_x_i_prev) + wd;
    Cex(idx_y_i, idx_x_i) = Cd;
    Qex(idx_y_i, idx_y_i) = Q;
    Rex(idx_u_i, idx_u_i) = R;
    
    mpc_ref_v(i + delay_step) = v_;
    
end


%% convex optimization

% The problem is to solve following for U.
%   1/2 * U'* mat1 * U + mat2 * U + C = 0
mat1 = Bex' * Cex' * Qex * Cex * Bex + Rex;
mat2 = (x0' * Aex' + Wex') * Cex' * Qex * Cex * Bex;

steering_rate_lim = param.mpc_constraint_steer_rate_deg * deg2rad;

if param.mpc_solve_without_constraint == true
    input_vec = -mat1 \ mat2';
else
    % --- convex optimization ---
    %   minimize for x, s.t.
    %   J(x) = 1/2 * x' * H * x + f' * x, 
    %   A*x <= b,   lb <= x <= ub

    H_ = (mat1 + mat1') / 2;
    f_ = mat2;
    
    % add steering rate constraint
    tmp = -eye(mpc_n-1, mpc_n);
    tmp(1:end,2:end) = tmp(1:end,2:end) + eye(mpc_n-1);
    T_ = kron(tmp, [0,0,1]) / mpc_dt;
    dsteer_vec_tmp_ = T_ * (Aex * x0 + Wex);
    A_ = [T_ * Bex; -T_ * Bex];    
    b_ = [steering_rate_lim * ones(mpc_n-1,1) - dsteer_vec_tmp_; steering_rate_lim * ones(mpc_n-1,1) + dsteer_vec_tmp_];

    % constraint for upper and lower steering boundary
    lb_ = -param.mpc_constraint_steering_deg * deg2rad * ones(mpc_n * DIM_U,1);
    ub_ = param.mpc_constraint_steering_deg * deg2rad * ones(mpc_n * DIM_U,1);
    options_ = optimoptions('quadprog', 'Algorithm','interior-point-convex', 'Display', 'off');
    [x_, fval, exitflag, output, lambda] = quadprog(H_, f_, A_, b_, [], [], lb_, ub_, [], options_);
    input_vec = x_;


    % for debug: compare with / without constraint optimization
    % input_vec_LS = -mat1 \ mat2';
    % figure(101);
    % plot(input_vec_LS); hold on;
    % plot(input_vec); grid on; hold off;
end

delta_des = input_vec(1);
v_des = ref_sp(IDX_VEL);
u = [v_des, delta_des];

% =================== for delay compensation ==============================
deltades_buffer = [delta_des; deltades_buffer(1:end-1)];
% =========================================================================

%% (debug) calculate predicted trajectory 

x_ = state;
predictd_states = zeros(length(input_vec), length(state));
for i = 1:length(input_vec)
    x_next = calc_kinematics_model(x_, input_vec(i), mpc_dt, mpc_ref_v(i), param.wheelbase, param.tau);
    predictd_states(i,:) = x_next';
    x_ = x_next;
end

predictd_states_vector = reshape(predictd_states, [], 1);

debug_ref_mat_no_delay_comp = debug_ref_mat(delay_step + 1:end, :);

predicted_error = Aex*x0 + Bex*input_vec + Wex;
predicted_error = transpose(reshape(predicted_error, 3, []));
predicted_state_ideal = debug_ref_mat_no_delay_comp(:,IDX_XY) + ...
    [-sin(debug_ref_mat_no_delay_comp(:,IDX_YAW)).*predicted_error(:,1), cos(debug_ref_mat_no_delay_comp(:,IDX_YAW)).*predicted_error(:,1)];

predicted_state_ideal = (reshape(predicted_state_ideal, [], 1));

debug_info = [input_vec', predictd_states_vector', predicted_state_ideal', error_lat];


% for debug 
% figure(1);plot(predicted_error(:,1),'b*-');
% figure(3);
% plot(input_vec); hold on;
% plot(predicted_error(:,3));hold on;
% plot(predictd_states(:,4)); hold off;


end

%% time varying error dynamics model
% used for error dynamics update
function [Ad, Bd, wd, Cd] = get_error_dynamics_state_matrix(dt, v, L, tau, curvature)
    
    % linearization around delta = 0
    % A = [0, v, 0;
    %     0, 0, v/L;
    %     0, 0, -1/tau];
    % B = [0; 0; 1/tau];
    % C = [1, 0, 0;
    %      0, 1, 0];
    % w = [0; 
    %     -v*curvature;
    %      0];

    % linearization around delta = delta_ref (better accuracy than delta=0)
    delta_r = atan(L*curvature);
    if (abs(delta_r) >= 40 /180 * pi)
        delta_r = (40 /180 * pi)*sign(delta_r);
    end
    cos_delta_r_squared_inv = 1 / ((cos(delta_r))^2);

    % difinition for continuous model
    A = [0, v, 0;
         0, 0, v/L*cos_delta_r_squared_inv;
         0, 0, -1/tau];
    B = [0; 0; 1/tau];
    C = [1, 0, 0;
         0, 1, 0];
    w = [0; 
        -v*curvature + v/L*(tan(delta_r)-delta_r*cos_delta_r_squared_inv);
         0];
    
    % discretization
    % Ad = eye(3) + A * dt;
    I = eye(3);
    Ad = (I - dt * 0.5 * A) \ (I + dt * 0.5 * A);
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