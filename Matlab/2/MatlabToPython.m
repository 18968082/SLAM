%%
%create the trajectory and lidar scans to be used for the python script.
% angles, trajectory, scans

clearvars -except trajectory angles scans

% ODOM x y theta tv rv accel
theta = 0;
tv = 0;
rv = 0;
accel = 0;
old_pose = [0,0];

fileID = fopen('Trajectory.txt','w');

for i=1:length(trajectory)
    if (trajectory(i,1) == old_pose(1)) && (trajectory(i,2) == old_pose(2))
        theta = theta - pi/2;
    end
    fprintf(fileID, 'ODOM %2.5f %2.5f %2.5f %f %f %f\n' ,trajectory(i,1),trajectory(i,2),theta,tv,rv,accel);
    old_pose = [trajectory(i,1),trajectory(i,2)];
end

fclose(fileID);
clearvars -except trajectory angles scans

% FLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta
x = 0;
y = 0;
theta = 0;
odom_x = 0;
odom_y = 0;
odom_theta = 0;

fileID = fopen('LidarScans_ranges.txt','w');

for i=1:length(scans)
    lidar = scans{i};
    fprintf(fileID, 'FLASER %d ',length(scans{i}.Ranges));
    for j=1:length(scans{i}.Ranges)
        fprintf(fileID, '%2.5f ' ,lidar.Ranges(j));
    end
    fprintf(fileID,'%f %f %f %f %f %f\n',x,y,theta,odom_x,odom_y,odom_theta);
end

fclose(fileID);
clearvars -except trajectory angles scans

%%
%The angles used for the Lidar Scans

% FLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta
x = 0;
y = 0;
theta = 0;
odom_x = 0;
odom_y = 0;
odom_theta = 0;

fileID = fopen('LidarScans_angles.txt','w');

fprintf(fileID, 'Angles ');
for i=1:length(angles)
    fprintf(fileID, '%1.10f ' ,angles(i));
end

fclose(fileID);
clearvars -except trajectory angles scans