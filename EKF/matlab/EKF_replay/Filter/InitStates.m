function [states, imu_start_index] = InitStates(param,imu_data,gps_data,mag_data,baro_data)

% constants
deg2rad = pi/180;

% initialise the state vector and quaternion
states = zeros(24,1);
quat = [1;0;0;0];

if (param.control.waitForGps == 1)
    % find IMU start index that coresponds to first valid GPS data
    %如果需要等待gps，先判断下是否有GPS数据
    %imu_data.time_us > gps_data.time_us(gps_data.start_index) 
    %find(arry,1,'first') 找出arry首次出现1个不为0的数
    %找出gps数据出现时候的imu的time_us
    imu_start_index = (find(imu_data.time_us > gps_data.time_us(gps_data.start_index), 1, 'first' ) - 50);
    %如果gps数据一致没有出现，就将imu赋值
    imu_start_index = max(imu_start_index,1);
else
    imu_start_index = 1;
end

% average first 100 accel readings to reduce effect of vibration
%del_vel：变化的速度值 
%accel_dt:只有1维，我猜他是加速度的总值，这里拿x,y,z三个方向的加速度分量与总的相/，有点归一化的感觉
initAccel(1) = mean(imu_data.del_vel(imu_start_index:imu_start_index+99,1))./mean(imu_data.accel_dt(imu_start_index:imu_start_index+99,1));
initAccel(2) = mean(imu_data.del_vel(imu_start_index:imu_start_index+99,2))./mean(imu_data.accel_dt(imu_start_index:imu_start_index+99,1));
initAccel(3) = mean(imu_data.del_vel(imu_start_index:imu_start_index+99,3))./mean(imu_data.accel_dt(imu_start_index:imu_start_index+99,1));

% align tilt using gravity vector (If the velocity is changing this will
% induce errors)
%利用重力向量对齐倾斜（如果速度改变将会导致误差）
quat = AlignTilt(quat,initAccel);%给四元数赋初值
states(1:4) = quat;

% add a roll, pitch, yaw mislignment 增加一个rpy的初始差值
quat_align_err = EulToQuat([param.control.rollAlignErr,param.control.pitchAlignErr,param.control.yawAlignErr]);%将欧拉角设置的错位误差转换为四元数的错位误差
quat = QuatMult(quat,quat_align_err);

% find magnetometer start index
mag_start_index = (find(mag_data.time_us > imu_data.time_us(imu_start_index), 1, 'first' ) - 5);
mag_start_index = max(mag_start_index,1);

% mean to reduce effect of noise in data
magBody(1,1) = mean(mag_data.field_ga(mag_start_index:mag_start_index+9,1));
magBody(2,1) = mean(mag_data.field_ga(mag_start_index:mag_start_index+9,2));
magBody(3,1) = mean(mag_data.field_ga(mag_start_index:mag_start_index+9,3));

% align heading and initialise the NED magnetic field states
quat = AlignHeading(quat,magBody,param.fusion.magDeclDeg*deg2rad);%通过磁场与重力加速度初始化了quat
states(1:4) = quat;

% initialise the NED magnetic field states
Tbn = Quat2Tbn(quat);
states(17:19) = Tbn*magBody;

if (param.control.waitForGps == 1)
    % initialise velocity and position using gps
    states(5:7) = gps_data.vel_ned(gps_data.start_index,:);
    states(8:9) = gps_data.pos_ned(gps_data.start_index,1:2);%GPS的高程并没有使用，使用的是气压计的高度观测值
else
    % initialise to be stationary at the origin
    states(5:7) = zeros(1,3);
    states(8:9) = zeros(1,2);
end

% find baro start index
baro_start_index = (find(baro_data.time_us > imu_data.time_us(imu_start_index), 1, 'first' ) - 10);
baro_start_index = max(baro_start_index,1);

% average baro data and initialise the vertical position
states(10) = -mean(baro_data.height(baro_start_index:baro_start_index+20));

end