    clear all;
close all;

% add required paths
addpath('../Common');

% load test data
load '../TestData/csv_data/11-7�ٷ��̼���������/baro_data.mat';
load '../TestData/csv_data/11-7�ٷ��̼���������/gps_data.mat';
load '../TestData/csv_data/11-7�ٷ��̼���������/imu_data.mat';
load '../TestData/csv_data/11-7�ٷ��̼���������/mag_data.mat';

% set parameters to default values
run('SetParameters.m');

% run the filter replay
output = RunFilter(param,imu_data,mag_data,baro_data,gps_data);

% generate and save output plots
runIdentifier = ' : PX4 data replay ';
folder = strcat('../OutputPlots/PX4');
PlotData(output,folder,runIdentifier);

% save output data
folder = '../OutputData/PX4';
fileName = '../OutputData/PX4/ekf_replay_output.mat';
if ~exist(folder,'dir')
    mkdir(folder);
end
save(fileName,'output');