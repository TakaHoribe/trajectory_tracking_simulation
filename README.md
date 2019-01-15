# trajectory_tracking_simulation
For developing trajectory tracking algorithm with MATLAB. 

Run /simulation/main.m to compute trajectory tracking simulation.

Reference path is designed in /path_design/path_design.m using spline from used-defined reference points.

・MPC

![mpc](https://raw.github.com/wiki/TakaHoribe/trajectory_tracking_simulation/images/mpc.gif)

・PID

![pid](https://raw.github.com/wiki/TakaHoribe/trajectory_tracking_simulation/images/pid.gif)

・pure pursuit

![purepursuit](https://raw.github.com/wiki/TakaHoribe/trajectory_tracking_simulation/images/purepursuit.gif)

## Currently available controller
* pure-pursuit
* PID
* MPC (use Optimization Toolbox)
* MPC without constraint

## Parameters
parameters are all set in /simulation/main.m

### simulation
* simulation_time : overall simulation time
* simulation_rk4_time_step : time span for integrate dynamics

### path
* vel_ref : reference speed [m/s]

### simulated vehicle model
* param.tau : steering dynamics 1d-approximated time constant [s]
* param.wheelbase : wheelbase length [m]
* param.steer_lim : tire angle limit [rad]
* param.vel_max : max velocity limit [m/s]
* param.vel_min : min velocity limit [m/s]
* param.input_delay : input delay [s]
* param.measurement_noise_stddev :  measurement noise standard deviation for [pos x[m], pos y[m], heading[rad], steering[rad]]
* param.steering_steady_state_error_deg : steady state error for steering model

### controllers
for all
* param.control_dt : control time span (zero order hold) [s]

for pure-pursuit
* param.pure_pursuit_lookahead : lookahead distance for pure-pursuit controller [m]

for mpc
* param.mpc_dt : time span for prediction
* param.mpc_n : step numbers for prediction
* param.mpc_constraint_steering_deg : steering limit constraint for optimization problem
* param.mpc_constraint_steer_rate_deg : steering rate limit constraint for optimization problem
* param.mpc_model_dim : mpc state model dimension
* param.mpc_Q : state weight matrix for optimization
* param.mpc_R : input weight matrix for optimization
* param.mpc_sensor_delay : for sensor delay compensation
