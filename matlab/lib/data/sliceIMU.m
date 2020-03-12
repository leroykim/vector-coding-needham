function cycles_imu = sliceIMU(peak_locs,imu_data)
    n_cycle = length(peak_locs)-1;
    cycles_imu = cell(n_cycle,1);

    for i = 1:n_cycle
        start_point = peak_locs(i);
        end_point = peak_locs(i+1);

        cycles_imu{i,1} = imu_data(start_point:end_point, :);
    end
end

