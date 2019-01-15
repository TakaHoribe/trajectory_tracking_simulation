set(0, 'defaultAxesFontSize', 14);
set(0, 'defaultTextFontSize', 14);
set(0, 'DefaultAxesLineWidth', 1.2, 'DefaultLineLineWidth', 1.5);

rad2deg = 180 / pi;
deg2rad = pi / 180;

lexus_param.tread_rear = 1.63;
lexus_param.tread_front = 1.64;
lexus_param.wheelbase = 2.79;

addpath func


%% 時間をそろえる
imuraw.t = imuraw.headerstampsecs + imuraw.headerstampnsecs * 1e-9;
wheelspeedrpt.t = wheelspeedrpt.headerstampsecs + wheelspeedrpt.headerstampnsecs * 1e-9;
yawraterpt.t = yawraterpt.headerstampsecs + yawraterpt.headerstampnsecs * 1e-9;
steerrpt.t = steerrpt.headerstampsecs + steerrpt.headerstampnsecs * 1e-9;
ndtpose.t = ndtpose.headerstampsecs + ndtpose.headerstampnsecs * 1e-9;
shiftrpt.t = shiftrpt.headerstampsecs + shiftrpt.headerstampnsecs * 1e-9;

tmp = max([imuraw.t(1), wheelspeedrpt.t(1), yawraterpt.t(1), steerrpt.t(1), ndtpose.t(1), shiftrpt.t(1)]);
imuraw.t = imuraw.t - tmp;
wheelspeedrpt.t = wheelspeedrpt.t - tmp;
yawraterpt.t = yawraterpt.t - tmp;
steerrpt.t = steerrpt.t - tmp;
ndtpose.t = ndtpose.t - tmp;
shiftrpt.t = shiftrpt.t - tmp;
ts = max([imuraw.t(1), wheelspeedrpt.t(1), yawraterpt.t(1), steerrpt.t(1), ndtpose.t(1), shiftrpt.t(1)]);
tf = min([imuraw.t(end), wheelspeedrpt.t(end), yawraterpt.t(end), steerrpt.t(end), ndtpose.t(end), shiftrpt.t(end)]);


%% 車速データからyawrateを求める

wheelspeed_all = [wheelspeedrpt.front_left_wheel_speed, wheelspeedrpt.front_right_wheel_speed, ...
            wheelspeedrpt.rear_left_wheel_speed, wheelspeedrpt.rear_right_wheel_speed];
        
figure;
plot(wheelspeedrpt.t, wheelspeed_all);grid on
xlabel('t [s]'); ylabel('vel [m/s]'); legend('front-left','front-right','rear-left','rear-right');



wheelspeedrpt.velsign = round(interp1q(shiftrpt.t, shiftrpt.output, wheelspeedrpt.t));
wheelspeedrpt.velsign(isnan(wheelspeedrpt.velsign) == 1) = 0;
wheelspeedrpt.velsign(wheelspeedrpt.velsign == 1) = -1;
wheelspeedrpt.velsign(wheelspeedrpt.velsign == 3) = 1;

wheelspeedrpt.yaw_from_rear_wheelspeeds = (wheelspeedrpt.rear_left_wheel_speed - wheelspeedrpt.rear_right_wheel_speed) /4 / lexus_param.tread_rear .* wheelspeedrpt.velsign;
wheelspeedrpt.yaw_from_front_wheelspeeds = (wheelspeedrpt.front_left_wheel_speed - wheelspeedrpt.front_right_wheel_speed) /4 / lexus_param.tread_front .* wheelspeedrpt.velsign;
wheelspeedrpt.vel_from_reat_wheelspeeds = (wheelspeedrpt.rear_left_wheel_speed + wheelspeedrpt.rear_right_wheel_speed) /2 .* wheelspeedrpt.velsign;


% figure;plot(imuraw.t, [imuraw.angular_velocityx, imuraw.angular_velocityy, imuraw.angular_velocityz]); grid on;

%% ステアリングからkinematicsモデルを用いて角速度を計算

% とりあえずプロット
figure;plot(steerrpt.t, [steerrpt.manual_input, steerrpt.command], steerrpt.t, steerrpt.output, '-.');grid on;
legend('manual', 'command','output');
xlabel('t [s]'), ylabel('steering [deg]');


% 線形補完 → 伝達関数推定：時定数＝1/8.7=0.115[s]
dt = 0.03;
tp = (ts:dt:tf)';
steer_p.cmd = interp1q(steerrpt.t, steerrpt.command, tp);
steer_p.out = interp1q(steerrpt.t, steerrpt.output, tp);


% ステアリング角度＋後輪車速から角速度を計算してodom角速度、imu角速度と比較
wheelspeed_p.vel_from_rear = interp1q(wheelspeedrpt.t, wheelspeedrpt.vel_from_reat_wheelspeeds, tp);
omega_steer_p = wheelspeed_p.vel_from_rear .* tan(steer_p.out * deg2rad) / lexus_param.wheelbase;



%% NDTの微分値から角速度を計算

figure;plot(ndtpose.posepositionx, ndtpose.posepositiony)

eul = quat2elu(ndtpose.poseorientationw, ndtpose.poseorientationx, ndtpose.poseorientationy, ndtpose.poseorientationz);
for i = 2:length(eul)
    if eul(i,3) - eul(i-1,3) > pi
        eul(i:end,3) = eul(i:end,3) - 2 * pi;
    elseif eul(i,3) - eul(i-1,3) < -pi
        eul(i:end,3) = eul(i:end,3) + 2 * pi;
    end
end

eul_p = interp1q(ndtpose.t, eul, tp);
eul_p_f = filtfilt(SOS, G, eul_p);
yawrate_from_ndt = (eul_p_f(2:end,3) - eul_p_f(1:end-1,3)) / dt;
yawrate_from_ndt = [yawrate_from_ndt; yawrate_from_ndt(end)];

%% ndtから求めた角速度に対してwheelbaseのパラメータをフィッティング

wheelbase_fitting = yawrate_from_ndt \ (wheelspeed_p.vel_from_rear .* tan(steer_p.out * deg2rad));
omega_steer_p_fitting = wheelspeed_p.vel_from_rear .* tan(steer_p.out * deg2rad) / wheelbase_fitting;


%% 全角速度推定データをプロット

figure;
plot(imuraw.t, imuraw.angular_velocityz * (-rad2deg)); hold on;
plot(yawraterpt.t, yawraterpt.yaw_rate); hold on;
plot(wheelspeedrpt.t, [wheelspeedrpt.yaw_from_front_wheelspeeds, wheelspeedrpt.yaw_from_rear_wheelspeeds] * rad2deg); hold on;
plot(tp, yawrate_from_ndt * rad2deg); hold on;
plot(tp, omega_steer_p * rad2deg); hold on;
plot(tp, omega_steer_p_fitting * rad2deg); grid on;
legend('imu -z', 'yawraterpt','front wheelspeed diff', 'rear wheelspeed diff', 'ndt deriv + LPF(3d-butter, fc=1Hz)', ...
    'kinematics model (theorical: L=2.79)', 'kinematics model (fitting: L=2.69)');
xlabel('t [s]'); ylabel('yawrate estimated [deg/s]')
