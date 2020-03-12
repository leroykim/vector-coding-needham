%% Plot IMU data

function plotIMU(imu)
    index = 1:size(imu,1);
    figure('Name', 'Sensor Data');
    
    axis(1) = subplot(3,1,1);
    hold on;
    plot(index, imu(:,1), 'r');
    plot(index, imu(:,2), 'g');
    plot(index, imu(:,3), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Index');
    ylabel('Acceleration (g)');
    title('Accelerometer');
    hold off;
    
    axis(2) = subplot(3,1,2);
    hold on;
    plot(index, imu(:,4), 'r');
    plot(index, imu(:,5), 'g');
    plot(index, imu(:,6), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Index');
    ylabel('Angular rate (degree/s)');
    title('Gyroscope');
    hold off;
    
    axis(3) = subplot(3,1,3);
    hold on;
    plot(index, imu(:,7), 'r');
    plot(index, imu(:,8), 'g');
    plot(index, imu(:,9), 'b');
    legend('X', 'Y', 'Z');
    xlabel('Index');
    ylabel('Flux (G)');
    title('Magnetometer');
    hold off;
    
    linkaxes(axis, 'x');
end