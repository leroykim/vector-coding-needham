function orientation_euler = getOrientationEstimate(...
                                        cycles,ave_length, n_cycle, AHRS)
                                    
    cycles_normalized = normalizeCycles(cycles, ave_length);
    
    orientation_quarternion = cell(n_cycle,1);
    orientation_euler = cell(n_cycle,1);
    
    for i = 1:n_cycle
        orientation_quarternion{i,1} = ...
            applyAHRS(cycles_normalized{i,1},AHRS);
    end
    
    for i = 1:n_cycle
        orientation_euler{i,1} = ...
            getEulerOrientation(orientation_quarternion{i,1});
    end
end

