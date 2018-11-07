fileFolder='.\11-1官方固件飞行数据\';

% %% convert baro data
% clear baro_data;
% last_time = 0;
% output_index = 1;
% for source_index = 1:length(timestamp)
%     baro_timestamp = timestamp(source_index) + baro_timestamp_relative(source_index);
%     if (baro_timestamp ~= last_time)
%         baro_data.time_us(output_index,1) = baro_timestamp;
%         baro_data.height(output_index) = baro_alt_meter(source_index);
%         last_time = baro_timestamp;
%         output_index = output_index + 1;
%     end
% end

%% convert IMU data to delta angles and velocities
% Note: these quatntis were converted from delta angles and velocites using
% the integral_dt values in the PX4 sensor module so we only need to
% multiply by integral_dt to convert back
%clear imu_data;
%赋值
imu=csvread([fileFolder 'log_376_2018-11-1-10-18-58_sensor_combined_0'],1,0);
timestamp=imu(:,1);
accelerometer_timestamp_relative=imu(:,6);
gyro_integral_dt=imu(:,5);
gyro_rad0=imu(:,2);
gyro_rad1=imu(:,3);
gyro_rad2=imu(:,4);
accelerometer_integral_dt=imu(:,10);
accelerometer_m_s20=imu(:,7);
accelerometer_m_s21=imu(:,8);
accelerometer_m_s22=imu(:,9);
%转换
n_samples = length(timestamp);
imu1_data.time_us = timestamp + accelerometer_timestamp_relative;
imu1_data.gyro_dt = gyro_integral_dt ./ 1e6;
imu1_data.del_ang = [gyro_rad0.*imu_data.gyro_dt, gyro_rad1.*imu_data.gyro_dt, gyro_rad2.*imu_data.gyro_dt];

imu1_data.accel_dt = accelerometer_integral_dt ./ 1e6;
imu1_data.del_vel = [accelerometer_m_s20.*imu_data.accel_dt, accelerometer_m_s21.*imu_data.accel_dt, accelerometer_m_s22.*imu_data.accel_dt];

% %% convert magnetomer data
% clear mag_data;
% last_time = 0;
% output_index = 1;
% for source_index = 1:length(timestamp)
%     mag_timestamp = timestamp(source_index) + magnetometer_timestamp_relative(source_index);
%     if (mag_timestamp ~= last_time)
%         mag_data.time_us(output_index,1) = mag_timestamp;
%         mag_data.field_ga(output_index,:) = [magnetometer_ga0(source_index),magnetometer_ga1(source_index),magnetometer_ga2(source_index)];
%         last_time = mag_timestamp;
%         output_index = output_index + 1;
%     end
% end

%% save data and clear workspace
%clearvars -except baro_data imu_data mag_data gps_data;
clearvars -except imu1_data;

%save baro_data.mat baro_data;
save imu_data.mat imu1_data;
%save mag_data.mat mag_data;
