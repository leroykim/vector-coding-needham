%% include external library
addpath('lib/data');
addpath('lib/plot');
addpath('lib/orientation');
addpath('lib/vector_coding');
addpath('lib/madgwick_algorithm_matlab');
addpath('lib/madgwick_algorithm_matlab/quaternion_library');

close all;                          % close all figures
clear;                             % clear all variables

%% IMU data directory metadata
data_dir = "../sample_data/gait_first";
imu_name_format = join([data_dir,"/", "*.csv"],'');
imu_list = dir(imu_name_format);

left_foot_data = strcat(data_dir,"/",imu_list(1).name);
right_foot_data = strcat(data_dir,"/",imu_list(2).name);
sacrum_data = strcat(data_dir,"/",imu_list(3).name);

%% importIMUData code
%
% Specification:
% 1:3 - accelerometer x,y,z
% 4:6 - gyroscope x,y,z
% 7:9 - magnetometer x,y,z

left_foot = readmatrix(left_foot_data);
right_foot = readmatrix(right_foot_data);
sacrum = readmatrix(sacrum_data);

%% Smoothing (optional)
for i = 1:9
    left_foot(:,i) = smooth(left_foot(:,i));
    right_foot(:,i) = smooth(right_foot(:,i));
    sacrum(:,i) = smooth(sacrum(:,i));
end

%% Plot imu data to see overall shape
plotIMU(left_foot);
plotIMU(right_foot);
plotIMU(sacrum);

%% Finding peaks

% Use this commented code below to check information about peak widths, 
% prominence, and etc.
%
% findpeaks(left_foot(:,6),...
%     'Annotate','extents','WidthReference','halfprom')
% findpeaks(left_foot(:,6),...
%     'Annotate','extents','WidthReference','halfheight')

% We will here look for toe-off using peak prominence
findpeaks(-left_foot(:,6),...
    'Annotate','extents','WidthReference','halfprom') % 222.4


% BECAREFUL!!
% When you specify a value for 'MinPeakDistance', the algorithm chooses 
% the tallest peak in the signal and ignores all peaks within 
% 'MinPeakDistance' of it. 
% The function then repeats the procedure for the tallest remaining peak 
% and iterates until it runs out of peaks to consider.

[~,peak_locs] = findpeaks(-left_foot(:,6),'MinPeakProminence', 200);

%% Get gait cycles using peak location
cycles_left_foot = sliceIMU(peak_locs,left_foot);
cycles_right_foot = sliceIMU(peak_locs,right_foot);
cycles_sacrum = sliceIMU(peak_locs,sacrum);

%% Get average cycle length and number of cycles
% Here, I choose left foot to calculate average cycle length
ave_cycle_length = getAverageCycleLength(cycles_left_foot);
n_cycle = length(cycles_left_foot);

%% Get orientation estimate
AHRS = MadgwickAHRS('SamplePeriod', 1/128, 'Beta', 0.1);

orientation_left_foot = ...
    getOrientationEstimate(...
        cycles_left_foot, ave_cycle_length, n_cycle, AHRS);
orientation_right_foot = ...
    getOrientationEstimate(...
        cycles_left_foot, ave_cycle_length, n_cycle, AHRS);
orientation_sacrum = ...
    getOrientationEstimate(...
        cycles_left_foot, ave_cycle_length, n_cycle, AHRS);