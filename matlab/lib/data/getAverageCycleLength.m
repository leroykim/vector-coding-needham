function average_cycle_length = getAverageCycleLength(imu_data)
    n_cycle = length(imu_data);                                                
    cycle_length_sum = 0;
    for i = 1:n_cycle
        cycle_length_sum = cycle_length_sum + length(imu_data{i,1});
    end
    average_cycle_length = round(cycle_length_sum / n_cycle);
end

