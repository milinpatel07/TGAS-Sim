%% Get the data from simulation
ego_acceleration = out.logsout.getElement('Ego Acceleration'); % acceleration of ego car
ego_acceleration_lc = out.logsout.getElement('lc_acceleration'); % acceleration of ego car
lateral_acceleration= out.logsout.getElement('lateral acceleration');
ego_velocity = out.logsout.getElement('ego_velocity'); % velocity of host car
driver_set_velocity = out.logsout.getElement('driver_set_velocity'); % driver-set velocity
lateral_velocity = out.logsout.getElement('lateral_velocity');
relative_distance = out.logsout.getElement('relative_distance'); % actual distance
relative_velocity = out.logsout.getElement('relative_velocity'); % relative velocity

safe_distance = (ego_velocity.Values.Data*time_gap) + default_spacing;
lead_velocity = out.logsout.getElement('lead_velocity');
 

% lead_velocity = relative_velocity.Values.Data + ego_velocity.Values.Data; % lead velocity

lateral_deviation = out.logsout.getElement('lateral_deviation');          % lateral deviation
relative_yaw_angle = out.logsout.getElement('relative_yaw_angle');        % relative yaw angle
steering_angle = out.logsout.getElement('Ego Steering Angle');                % steering angle
steering_angle_lc = out.logsout.getElement('lc_steering angle'); 
tmax = ego_velocity.Values.time(end);

Collision = out.logsout.getElement('collision');      % collision

lange_change_enable = out.logsout.getElement('lc_signal');


%% Plot the spacing control results

figure('Name','Longitudinal Control Performance','position',[100 100 720 600])

% velocity

subplot(3,1,1)
plot(ego_velocity.Values.time,ego_velocity.Values.Data,'b',...
    driver_set_velocity.Values.time,driver_set_velocity.Values.Data,'k--',...
    lead_velocity.Values.time,lead_velocity.Values.Data,'r')

hold on
ylim([18,36])
xlim([0,tmax])
grid on
legend('Ego Velocity','ACC Set Velocity','Lead Car Velocity','location','NorthEast')
title('Longitudinal Speed')
xlabel('Time (sec)')
ylabel('Ego Velocity m/s')
% 

% distance
subplot(3,1,2)
plot(relative_distance.Values.time,relative_distance.Values.Data,'r')
hold on
% plot(safe_distance.Values.time,safe_distance.Values.Data,'b')
plot(relative_distance.Values.time,safe_distance,'b')
%plot(ClearanceStatus.Values.time,ClearanceStatus.Values.Data,'g')
grid on
xlim([0,tmax])
legend('relative distance','safe distance','location','NorthEast')
title('Distance between two cars')
xlabel('time (sec)')
ylabel('Distance m')

% acceleration
subplot(3,1,3)
plot(ego_acceleration.Values.time,ego_acceleration.Values.Data,'r')
% plot(ego_acceleration_lc.Values.time,ego_acceleration_lc.Values.Data,'k')

grid on
xlim([0,tmax])
ylim([-3.5,2.5])
legend('Ego Acceleration','location','NorthEast')
title('Acceleration')
xlabel('time (sec)')
ylabel('Longitudinal Acceleration $m/s^2$','Interpreter','latex')



sgt = sgtitle('Longitudinal Control Performance','Color','blue');
sgt.FontSize = 20;


%% Plot the lane following results
figure('Name','Lateral Control Performance','position',[835 100 720 600])

% lateral deviation
subplot(3,1,1)
plot(lateral_deviation.Values,'b')
grid on
xlim([0,tmax])
ylim([-0.6,0.6])
legend('lateral deviation','location','NorthEast')
title('Lateral deviation')
xlabel('time (sec)')
ylabel('lateral deviation (m)')

% relative yaw angle
subplot(3,1,2)
plot(relative_yaw_angle.Values,'b')
grid on
xlim([0,tmax])
legend('relative yaw angle','location','NorthEast')
title('Relative yaw angle')
xlabel('time (sec)')
ylabel('relative yaw angle (rad)')

% steering angle
subplot(3,1,3)
%  plot(steering_angle.Values.time,steering_angle.Values.Data,'b')
plot(steering_angle_lc.Values.time,steering_angle_lc.Values.Data,'b');
grid on
xlim([0,tmax])
ylim([-0.2,0.2])
legend('steering angle','location','SouthEast')
title('Steering angle')
xlabel('time (sec)')
ylabel('steering angle (rad)')

sgt = sgtitle('Lateral Control Performance','Color','blue');
sgt.FontSize = 20;

figure('Name','Lateral Control Performance','position',[835 100 720 600])
 %lateral deviation
subplot(2,1,1)
plot(lateral_acceleration.Values,'b')
grid on
xlim([0,tmax])
ylim([-1.5,1.5])
legend('Lateral Acceleration','location','NorthEast')
title('Lateral Acceleration')
xlabel('Time (sec)')
ylabel('Lateral Acceleration $m/s^2$','Interpreter','latex')

subplot(2,1,2)
plot(lateral_velocity.Values.time,lateral_velocity.Values.Data,'b')
grid on
xlim([0,tmax])
ylim([-5,5])
legend('Lateral Velocity','location','NorthEast')
title('Lateral Velocity')
xlabel('Time (sec)')
ylabel('Lateral Velocity')

% end