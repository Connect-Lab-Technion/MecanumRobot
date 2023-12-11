%% Plots

set(groot , 'defaultAxesFontName' , 'Latin Modern Math')
set(groot , 'defaultAxesFontSize' , 12)

%% Comparison of Z angular velocity data

plot(out.Zinput.Time,out.Zinput.Data)
grid minor
xlabel("Time [sec]")
ylabel("Z angular velocity [rad/s]")
ylim([-5 15])

hold on
plot(out.vel_raw_Z_angular_velocity_output.Time , -out.vel_raw_Z_angular_velocity_output.Data , 'r')
plot(out.odom_raw_Z_angular_velocity_output.Time , -out.odom_raw_Z_angular_velocity_output.Data , '--g')
plot(out.imu_raw_Z_angular_velocity_output.Time , -out.imu_raw_Z_angular_velocity_output.Data , '--y')
plot(out.imu_data_Z_angular_velocity_output.Time , -out.imu_data_Z_angular_velocity_output.Data , '--m')

legend('/cmd.vel input' , '/vel.raw output', '/odom.raw output' ,'/imu/imu.raw output' , '/imu/imu.data output')
title('Step response for Z angular velocity')
subtitle('K_P = 3.3  K_I = 1  K_D = 0.1')

%% Comparison of Y linear velocity data

plot(out.Yinput.Time,out.Yinput.Data)
grid minor
xlabel("Time [sec]")
ylabel("Y linear velocity [m/s]")
ylim([-5 15])

hold on
plot(out.vel_raw_Y_linear_velocity_output.Time , -out.vel_raw_Y_linear_velocity_output.Data , 'r')
plot(out.odom_raw_Y_linear_velocity_output.Time , -out.odom_raw_Y_linear_velocity_output.Data , '--g')


legend('/cmd.vel input' , '/vel.raw output', '/odom.raw output')
title('Step response for Y linear velocity')
subtitle('K_P = 3.3  K_I = 1  K_D = 0.1')
%% Comparison of X linear velocity data

plot(out.Xinput.Time,out.Xinput.Data)
grid minor
xlabel("Time [sec]")
ylabel("X linear velocity [m/s]")
ylim([-5 15])

hold on
plot(out.vel_raw_X_linear_velocity_output.Time , -out.vel_raw_X_linear_velocity_output.Data , 'r')
plot(out.odom_raw_X_linear_velocity_output.Time , -out.odom_raw_X_linear_velocity_output.Data , '--g')

legend('/cmd.vel input' , '/vel.raw output', '/odom.raw output')
title('Step response for X linear velocity')
subtitle('K_P = 3.3  K_I = 1  K_D = 0.1')
