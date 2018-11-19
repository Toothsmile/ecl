clear all;
close all;

% add required paths
addpath('../Common');

% load test data
load '../TestData/csv_data/11-7-Office-testMag-Data/baro_data.mat';
load '../TestData/csv_data/11-7-Office-testMag-Data/gps_data.mat';
load '../TestData/csv_data/11-7-Office-testMag-Data/imu_data.mat';
load '../TestData/csv_data/11-7-Office-testMag-Data/mag_data.mat';

% set parameters to default values
run('SetParameters.m');

% run the filter replay
output = RunFilter(param,imu_data,mag_data,baro_data,gps_data);

% generate and save output plots
runIdentifier = ' : PX4 data replay ';
folder = strcat('../OutputPlots/11-7-Office-testMag-Data-PX4');
PlotData(output,folder,runIdentifier);

% save output data
folder = '../OutputData/11-7-Office-testMag-DataPX4';
fileName = '../OutputData/11-7-Office-testMag-DataPX4/ekf_replay_output.mat';
if ~exist(folder,'dir')
    mkdir(folder);
end
save(fileName,'output');