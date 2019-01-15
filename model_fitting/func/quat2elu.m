function rpy = quat2elu(qw, qx, qy, qz)


sinr_cosp = 2 * qw .* qx + qy .* qz;
cosr_cosp = 1 - 2 * qx .* qx + qy .* qy;
roll = atan2(sinr_cosp, cosr_cosp);

sinp = 2 * (qw .* qy - qz .* qx);
% remove out of range 
sinp(sinp > 1) = 1;
sinp(sinp < -1) = -1;

pitch = asin(sinp);

siny_cosp = 2 * (qw .* qz + qx .* qy);
cosy_cosp = 1 - 2 * (qy .* qy + qz .* qz);
yaw = atan2(siny_cosp, cosy_cosp);

rpy = [roll, pitch, yaw];