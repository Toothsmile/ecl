
file_tmp='./11-7 test mag_data/02_02_48';
% %% convert baro data
clear baro_data;
baro=csvread([file_tmp '_vehicle_air_data_0.csv'],1,0);
%赋值
timestamp=baro(:,1);
baro_alt_meter=baro(:,2);
last_time = 0;
output_index = 1;
for source_index = 1:length(timestamp)
     %baro_timestamp = timestamp(source_index) + baro_timestamp_relative(source_index);
     baro_timestamp = timestamp(source_index);
    if (baro_timestamp ~= last_time)
        baro_data.time_us(output_index,1) = baro_timestamp;
        baro_data.height(output_index) = baro_alt_meter(source_index);%height data 怎么是横的数据
        last_time = baro_timestamp;
        output_index = output_index + 1;
    end
end

%% convert IMU data to delta angles and velocities
% Note: these quatntis were converted from delta angles and velocites using
% the integral_dt values in the PX4 sensor module so we only need to
% multiply by integral_dt to convert back
clear imu_data;
%赋值
imu=csvread([file_tmp '_sensor_combined_0.csv'],1,0);
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
imu_data.time_us = timestamp + accelerometer_timestamp_relative;
imu_data.gyro_dt = gyro_integral_dt ./ 1e6;
imu_data.del_ang = [gyro_rad0.*imu_data.gyro_dt, gyro_rad1.*imu_data.gyro_dt, gyro_rad2.*imu_data.gyro_dt];

imu_data.accel_dt = accelerometer_integral_dt ./ 1e6;
imu_data.del_vel = [accelerometer_m_s20.*imu_data.accel_dt, accelerometer_m_s21.*imu_data.accel_dt, accelerometer_m_s22.*imu_data.accel_dt];

% %% convert magnetomer data
clear mag_data;
mag=csvread([file_tmp '_vehicle_magnetometer_0.csv'],1,0);
%赋值 
timestamp=mag(:,1);
magnetometer_ga0=mag(:,2);
magnetometer_ga1=mag(:,3);
magnetometer_ga2=mag(:,4);
last_time = 0;
output_index = 1;
for source_index = 1:length(timestamp)
    mag_timestamp = timestamp(source_index);
    if (mag_timestamp ~= last_time)
        mag_data.time_us(output_index,1) = mag_timestamp;
        mag_data.field_ga(output_index,:) = [magnetometer_ga0(source_index),magnetometer_ga1(source_index),magnetometer_ga2(source_index)];
        last_time = mag_timestamp;
        output_index = output_index + 1;
    end
end

save baro_data.mat baro_data;
save imu_data.mat imu_data;
save mag_data.mat mag_data;
%convert GPS csv

clear gps_data;
gps=csvread([file_tmp '_vehicle_gps_position_0.csv'],1,0);
%赋值
timestamp=gps(:,1);
timestamp_time_relative=gps(:,20);
eph=gps(:,9);
s_variance_m_s=gps(:,7);
epv=gps(:,10);
lat=gps(:,3);
lon=gps(:,4);
alt=gps(:,5);
vel_n_m_s=gps(:,15);
vel_e_m_s=gps(:,16);
vel_d_m_s=gps(:,17);
fix_type=gps(:,21);

gps_data.time_us = timestamp + timestamp_time_relative;
gps_data.pos_error = eph;
gps_data.spd_error = s_variance_m_s;
gps_data.hgt_error = epv;

% set reference point used to set NED origin when GPS accuracy is sufficient
gps_data.start_index = max(min(find(gps_data.pos_error < 5.0)),min(find(gps_data.spd_error < 1.0)));
if isempty(gps_data.start_index)
    gps_data.start_index = 1;
    gps_data.refLLH = [1e-7*median(lat);1e-7*median(lon);0.001*median(alt)];
else
    gps_data.refLLH = [1e-7*lat(gps_data.start_index);1e-7*lon(gps_data.start_index);0.001*alt(gps_data.start_index)];
end

% convert GPS data to NED
for index = 1:length(timestamp)
    if (fix_type(index) >= 3)
        gps_data.pos_ned(index,:) = LLH2NED([1e-7*lat(index);1e-7*lon(index);0.001*alt(index)],gps_data.refLLH);
        gps_data.vel_ned(index,:) = [vel_n_m_s(index),vel_e_m_s(index),vel_d_m_s(index)];
    else
        gps_data.pos_ned(index,:) = [0,0,0];
        gps_data.vel_ned(index,:) = [0,0,0];
    end
end



%% save data and clear workspace
%clearvars -except baro_data imu_data mag_data gps_data;
clearvars -except baro_data imu_data mag_data gps_data;

save baro_data.mat baro_data;
save imu_data.mat imu_data;
save mag_data.mat mag_data;
save gps_data.mat;
