%declare name of the bag
experimentbag = rosbag('2020-03-23-16-17-42.bag')
desiredheading = select(experimentbag, "Topic", '/guidance/desired_heading');
desiredheadingts = timeseries(desiredheading, 'Data');
start_time = desiredheadingts.get.TimeInfo.Start;
%heading plot
heading = select(experimentbag, "Topic", 'vectornav/ins_2d/NED_pose');
headingts = timeseries(heading, 'Theta');
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
figure
plot(t,headingdata)
hold on
%desired heading plot
desiredheading = select(experimentbag, "Topic", '/guidance/desired_heading');
desiredheadingts = timeseries(desiredheading, 'Data');
t = desiredheadingts.get.Time - start_time;
desiredheadingdata = desiredheadingts.get.Data;
plot(t,desiredheadingdata)
hold off
legend('$\psi$','$\psi_{d}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\psi$ [rad]', 'Interpreter', 'latex')
%right thruster plot
right = select(experimentbag, "Topic", '/usv_control/controller/right_thruster');
rightts = timeseries(right, 'Data');
t = rightts.get.Time - start_time;
rightdata = rightts.get.Data;
figure
plot(t,rightdata)
hold on
%left thruster plot
left = select(experimentbag, "Topic", '/usv_control/controller/left_thruster');
leftts = timeseries(left, 'Data');
t = leftts.get.Time - start_time;
leftdata = leftts.get.Data;
plot(t,leftdata)
hold off
legend('$T_{stbd}$','$T_{port}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('Thrust [N]', 'Interpreter', 'latex') 
%speed plot
speed = select(experimentbag, "Topic", '/vectornav/ins_2d/local_vel');
speedts = timeseries(speed, 'X');
t = speedts.get.Time - start_time;
speeddata = speedts.get.Data;
figure
plot(t,speeddata)
hold on
%desired speed plot
desiredspeed = select(experimentbag, "Topic", '/guidance/desired_speed');
desiredspeedts = timeseries(desiredspeed, 'Data');
t = desiredspeedts.get.Time - start_time;
desiredspeeddata = desiredspeedts.get.Data;
plot(t,desiredspeeddata)
hold off
legend('$u$','$u_{d}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m/s]', 'Interpreter', 'latex') 