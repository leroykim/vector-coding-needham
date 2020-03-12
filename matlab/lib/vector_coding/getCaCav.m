function ca_cav = getCaCav(imu_la, imu_ra, imu_lw, ...
    imu_rw, imu_sc, peak_locs, path)
    
    %GETCACAV Summary of this function goes here
    %   Detailed explanation goes here

    %% Slice signals cycle by cycle
    cycles_la = sliceCycles(peak_locs,imu_la);
    cycles_ra = sliceCycles(peak_locs,imu_ra);
    cycles_lw = sliceCycles(peak_locs,imu_lw);
    cycles_rw = sliceCycles(peak_locs,imu_rw);
    cycles_sc = sliceCycles(peak_locs,imu_sc);


    %% Cycle length normalization
    % Get average cycle length
    average_cycle_length = getAverageCycleLength(cycles_la);

    cycles_normalized_la = normalizeCycles(cycles_la, average_cycle_length);
    cycles_normalized_ra = normalizeCycles(cycles_ra, average_cycle_length);
    cycles_normalized_lw = normalizeCycles(cycles_lw, average_cycle_length);
    cycles_normalized_rw = normalizeCycles(cycles_rw, average_cycle_length);
    cycles_normalized_sc = normalizeCycles(cycles_sc, average_cycle_length);

    %% Process sensor data through algorithm
    n_cycle = length(cycles_la);

    AHRS = MadgwickAHRS('SamplePeriod', 1/128, 'Beta', 0.1);
    orientation_quarternion_la = cell(n_cycle,1);
    orientation_quarternion_ra = cell(n_cycle,1);
    orientation_quarternion_lw = cell(n_cycle,1);
    orientation_quarternion_rw = cell(n_cycle,1);
    orientation_quarternion_sc = cell(n_cycle,1);
    for i = 1:n_cycle
        orientation_quarternion_la{i,1} = applyAHRS(...
                                                cycles_normalized_la{i,1},...
                                                AHRS);
        orientation_quarternion_ra{i,1} = applyAHRS(...
                                                cycles_normalized_ra{i,1},...
                                                AHRS);
        orientation_quarternion_lw{i,1} = applyAHRS(...
                                                cycles_normalized_lw{i,1},...
                                                AHRS);
        orientation_quarternion_rw{i,1} = applyAHRS(...
                                                cycles_normalized_rw{i,1},...
                                                AHRS);
        orientation_quarternion_sc{i,1} = applyAHRS(...
                                                cycles_normalized_sc{i,1},...
                                                AHRS);

    end

    %% Get Euler angle from quaternion result
    orientation_euler_la = cell(n_cycle,1);
    orientation_euler_ra = cell(n_cycle,1);
    orientation_euler_lw = cell(n_cycle,1);
    orientation_euler_rw = cell(n_cycle,1);
    orientation_euler_sc = cell(n_cycle,1);
    for i = 1:n_cycle
        orientation_euler_la{i,1} = ...
            getEulerOrientation(orientation_quarternion_la{i,1});
        orientation_euler_ra{i,1} = ...
            getEulerOrientation(orientation_quarternion_ra{i,1});
        orientation_euler_lw{i,1} = ...
            getEulerOrientation(orientation_quarternion_lw{i,1});
        orientation_euler_rw{i,1} = ...
            getEulerOrientation(orientation_quarternion_rw{i,1});
        orientation_euler_sc{i,1} = ...
            getEulerOrientation(orientation_quarternion_sc{i,1});
    end

    %% Coupling angle (CA)
    ca_la = ...
        getCouplingAngle(orientation_euler_la,orientation_euler_sc);
    ca_ra = ...
        getCouplingAngle(orientation_euler_ra,orientation_euler_sc);
    ca_lw = ...
        getCouplingAngle(orientation_euler_lw,orientation_euler_sc);
    ca_rw = ...
        getCouplingAngle(orientation_euler_rw,orientation_euler_sc);

    %% Average coupling angle
    % x_bar = average of x
    % y_bar = average of y
    %
    % r = length of couplilng angle
    % cav = coupling angle variability
    [x_bar_la, y_bar_la] = getAverageComponent(ca_la);
    [x_bar_ra, y_bar_ra] = getAverageComponent(ca_ra);
    [x_bar_lw, y_bar_lw] = getAverageComponent(ca_lw);
    [x_bar_rw, y_bar_rw] = getAverageComponent(ca_rw);

    % Average coupling angle
    ca_la = rad2deg(...
        getAverageCouplingAngle(x_bar_la, y_bar_la));
    ca_ra = rad2deg(...
        getAverageCouplingAngle(x_bar_ra, y_bar_ra));
    ca_lw = rad2deg(...
        getAverageCouplingAngle(x_bar_lw, y_bar_lw));
    ca_rw = rad2deg(...
        getAverageCouplingAngle(x_bar_rw, y_bar_rw));

    %% length of average coupling angle & coupling angle variabilty
    sample_count = length(x_bar_la);

    r_la = zeros(sample_count, 1);
    r_ra = zeros(sample_count, 1);
    r_lw = zeros(sample_count, 1);
    r_rw = zeros(sample_count, 1);

    cav_la = zeros(sample_count, 1);
    cav_ra = zeros(sample_count, 1);
    cav_lw = zeros(sample_count, 1);
    cav_rw = zeros(sample_count, 1);

    for k = 1:sample_count
        r_la(k) = sqrt(x_bar_la(k)^2+y_bar_la(k)^2);
        r_ra(k) = sqrt(x_bar_ra(k)^2+y_bar_ra(k)^2);
        r_lw(k) = sqrt(x_bar_lw(k)^2+y_bar_lw(k)^2);
        r_rw(k) = sqrt(x_bar_rw(k)^2+y_bar_rw(k)^2);

        cav_la(k) = arrayfun(@(x) sqrt(2*(1-x)),r_la(k));
        cav_ra(k) = arrayfun(@(x) sqrt(2*(1-x)),r_ra(k));
        cav_lw(k) = arrayfun(@(x) sqrt(2*(1-x)),r_lw(k));
        cav_rw(k) = arrayfun(@(x) sqrt(2*(1-x)),r_rw(k));
    end

    if exist('path', 'var')
        ca_cav_arr = cat(2, ca_la, ca_ra, ca_lw, ca_rw,...
                    cav_la, cav_ra, cav_lw, cav_rw);
        column_names = {'ca_la', 'ca_ra', 'ca_lw', 'ca_rw',...
                        'cav_la', 'cav_ra', 'cav_lw', 'cav_rw'};
        ca_cav_tab = array2table(ca_cav_arr,'VariableNames', column_names);
        
        path = strcat(path, "-CaCav.csv");
        writetable(ca_cav_tab, path, 'Delimiter',',');
    end
    
    ca_cav{1,1} = ca_la;
    ca_cav{1,2} = cav_la;
    
    ca_cav{2,1} = ca_ra;
    ca_cav{2,2} = cav_ra;
    
    ca_cav{3,1} = ca_lw;
    ca_cav{3,2} = cav_lw;
    
    ca_cav{4,1} = ca_rw;
    ca_cav{4,2} = cav_rw;

end

