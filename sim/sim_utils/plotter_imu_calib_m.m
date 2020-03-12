close all
clc

figure
hold on
plot([timeline(1) timeline(end)], [imu_quat_shift(2) imu_quat_shift(2)], 'r--')
plot([timeline(1) timeline(end)], [imu_quat_shift(3) imu_quat_shift(3)], 'g--')
plot([timeline(1) timeline(end)], [imu_quat_shift(4) imu_quat_shift(4)], 'b--')

plot(timeline, est_states(8, :), 'r')
plot(timeline, est_states(9, :), 'g')
plot(timeline, est_states(10, :), 'b')

title('q')

figure
hold on
plot([timeline(1) timeline(end)], [imu_acc_scale_factor(1,1) imu_acc_scale_factor(1,1)], 'r--')
plot([timeline(1) timeline(end)], [imu_acc_scale_factor(2,2) imu_acc_scale_factor(2,2)], 'g--')
plot([timeline(1) timeline(end)], [imu_acc_scale_factor(3,3) imu_acc_scale_factor(3,3)], 'b--')

plot(timeline, est_states(1, :), 'r')
plot(timeline, est_states(2, :), 'g')
plot(timeline, est_states(3, :), 'b')

title('s')

figure
hold on
plot([timeline(1) timeline(end)], [imu_acc_bias(1) imu_acc_bias(1)], 'r--')
plot([timeline(1) timeline(end)], [imu_acc_bias(2) imu_acc_bias(2)], 'g--')
plot([timeline(1) timeline(end)], [imu_acc_bias(3) imu_acc_bias(3)], 'b--')

plot(timeline, est_states(4, :), 'r')
plot(timeline, est_states(5, :), 'g')
plot(timeline, est_states(6, :), 'b')

title('e')