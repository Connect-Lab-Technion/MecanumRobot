%% Plots

set(groot , 'defaultAxesFontName' , 'Latin Modern Math')
set(groot , 'defaultAxesFontSize' , 12)

%% Comparison of Z angular velocity data

plot(out.Zinput.Time,out.Zinput.Data)
grid minor
xlabel("Time [sec]")
ylabel("Z angular velocity [rad/s]")
ylim([0 10])

hold on
%plot(out.vel_raw_Z_angular_velocity_output.Time , -out.vel_raw_Z_angular_velocity_output.Data , 'r')
plot(out.raw_odom_Z_angular_velocity_output.Time , -out.raw_odom_Z_angular_velocity_output.Data , '--g')
plot(out.data_raw_Z_angular_velocity_output.Time , out.data_raw_Z_angular_velocity_output.Data , '--y')
plot(out.data_Z_angular_velocity_output.Time , out.data_Z_angular_velocity_output.Data , '--m')

legend('/cmd.vel input' , '/raw.odom output' , '/imu/data.raw output' , '/imu/data output')
title('Comparison of Z angular velocity data')

%% Comparison of Y linear velocity data

plot(out.Yinput.Time,out.Yinput.Data)
grid minor
xlabel("Time [sec]")
ylabel("X linear velocity [m/s]")
ylim([-5 15])

hold on
plot(out.vel_raw_Y_linear_velocity_output.Time , -out.vel_raw_Y_linear_velocity_output.Data , 'r')
plot(out.odom_raw_Y_linear_velocity_output.Time , -out.odom_raw_Y_linear_velocity_output.Data , '--g')


legend('/cmd.vel input' , '/vel.raw output' , '/odom.raw output')
title('Comparison of Y linear velocity data')