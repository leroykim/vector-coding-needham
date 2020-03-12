function imu_quaternion = applyAHRS(imu_data, AHRS)
%% Caution
%
% Sequence of the arguments in AHRS.Update *must be* gyro, acc, mag
%
    [rownum, ~] = size(imu_data);
    imu_quaternion = zeros(rownum, 4);
    for t = 1:rownum
        % gyroscope units must be radians
        AHRS.Update(imu_data(t,4:6) * (pi/180), ...
            imu_data(t,1:3), ...
            imu_data(t,7:9));
        imu_quaternion(t, :) = AHRS.Quaternion;
    end
end

