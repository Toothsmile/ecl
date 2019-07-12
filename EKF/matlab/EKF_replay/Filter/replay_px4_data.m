clear all;
close all;

% add required paths
addpath('../Common');
% record heading
% load test data
load '../TestData/benkebishe/baro_data.mat';
load '../TestData/benkebishe/gps_data.mat';
load '../TestData/benkebishe/imu_data.mat';
load '../TestData/benkebishe/mag_data.mat';

% set parameters to default values
run('SetParameters.m');

% run the filter replay
output = RunFilter(param,imu_data,mag_data,baro_data,gps_data);

% generate and save output plots
runIdentifier = ' : PX4 data replay ';
folder = strcat('../OutputPlots/rtk');
PlotData(output,folder,runIdentifier);

% save output data
folder = '../OutputData/11-7-Office-testMag-DataPX4';
fileName = '../OutputData/11-7-Office-testMag-DataPX4/ekf_replay_output.mat';
if ~exist(folder,'dir')
    mkdir(folder);
end
save(fileName,'output');

%by sjj
counter=1:length(output.position_NED);
figure,plot(gps_data.pos_ned(:,1),gps_data.pos_ned(:,2),'.');
figure,plot(output.position_NED(:,1),output.position_NED(:,2),'.',gps_data.pos_ned(:,1),gps_data.pos_ned(:,2),'.');
figure();
plot(sqrt(output.state_variances(:,8)),'.');
title('水平精度');
figure();
plot(sqrt(output.state_variances(:,10)),'.');
title('竖直精度');

%save NED data 

localNED=[output.time_lapsed',output.position_NED];
save local_road_NED.mat localNED ;