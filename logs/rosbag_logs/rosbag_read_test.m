clc
clear
close all
format long

%% read file
path = 'logs/rosbag_logs/oct_01_2020/2020-10-01-18-35-40.bag';
bag = rosbag(path);
bag.AvailableTopics

%% gnns

% base fix
selection = select(bag,'Time',[bag.StartTime bag.StartTime + 1],'Topic','/gnss_base/fix');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};

lat_base = cellfun(@(m) double(m.Latitude), msgs);
lon_base = cellfun(@(m) double(m.Longitude), msgs);
alt_base = cellfun(@(m) double(m.Altitude), msgs);
type_base = cellfun(@(m) double(m.PositionCovarianceType), msgs);

% base vel
selection = select(bag,'Time',[bag.StartTime bag.StartTime + 1],'Topic','/gnss_base/vel');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};

vx_base = cellfun(@(m) double(m.Vector.X), msgs);
vy_base = cellfun(@(m) double(m.Vector.Y), msgs);
vz_base = cellfun(@(m) double(m.Vector.Z), msgs);

% left fix
selection = select(bag,'Time',[bag.StartTime bag.StartTime + 1],'Topic','/gnss_left/fix');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};

lat_left = cellfun(@(m) double(m.Latitude), msgs);
lon_left = cellfun(@(m) double(m.Longitude), msgs);
alt_left = cellfun(@(m) double(m.Altitude), msgs);
type_left = cellfun(@(m) double(m.PositionCovarianceType), msgs);

% right fix
selection = select(bag,'Time',[bag.StartTime bag.StartTime + 1],'Topic','/gnss_right/fix');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};

lat_right = cellfun(@(m) double(m.Latitude), msgs);
lon_right = cellfun(@(m) double(m.Longitude), msgs);
alt_right = cellfun(@(m) double(m.Altitude), msgs);
type_right = cellfun(@(m) double(m.PositionCovarianceType), msgs);

% left dr ecef
selection = select(bag,'Time',[bag.StartTime bag.StartTime + 1],'Topic','/gnss_left/dr_ecef');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};

drx_left = cellfun(@(m) double(m.Vector.X), msgs);
dry_left = cellfun(@(m) double(m.Vector.Y), msgs);
drz_left = cellfun(@(m) double(m.Vector.Z), msgs);

% right dr ecef
selection = select(bag,'Time',[bag.StartTime bag.StartTime + 1],'Topic','/gnss_right/dr_ecef');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1};

drx_right = cellfun(@(m) double(m.Vector.X), msgs);
dry_right = cellfun(@(m) double(m.Vector.Y), msgs);
drz_right = cellfun(@(m) double(m.Vector.Z), msgs);

% imu
selection = select(bag,'Time',[bag.StartTime bag.StartTime + 1],'Topic','/icm20608');
msgs = readMessages(selection,'DataFormat','struct');
msgs{1}

ax = cellfun(@(m) double(m.LinearAcceleration.X), msgs);
ay = cellfun(@(m) double(m.LinearAcceleration.Y), msgs);
az = cellfun(@(m) double(m.LinearAcceleration.Z), msgs);

wx = cellfun(@(m) double(m.AngularVelocity.X), msgs);
wy = cellfun(@(m) double(m.AngularVelocity.Y), msgs);
wz = cellfun(@(m) double(m.AngularVelocity.Z), msgs);

%%
phi = lat_base(1);
lambda = lon_base(1);
h = alt_base(1);
[x0, y0, z0] = wgs2ecef(phi, lambda, h);
N = min([length(lat_base), length(vx_base), length(drx_left), length(dry_left)])
for i = 1:N
    phi = lat_base(i);
    lambda = lon_base(i);
    h = alt_base(i);
    [x, y, z] = wgs2ecef(phi, lambda, h);
    x_base_ecef(i,1) = x;
    y_base_ecef(i,1) = y;
    z_base_ecef(i,1) = z;
    
    R_ecef_enu = rmx2enu(phi, lambda);
    R = R_ecef_enu';
    r_base_enu(:,i) = R*[(x-x0);(y-y0);(z-z0)];
    
    x_left_ecef(i,1) = x_base_ecef(i,1) + drx_left(i);
    y_left_ecef(i,1) = y_base_ecef(i,1) + dry_left(i);
    z_left_ecef(i,1) = z_base_ecef(i,1) + drz_left(i);
    
    r_left_enu(:,i) = R*([x_left_ecef(i,1) - x0; y_left_ecef(i,1) - y0; z_left_ecef(i,1) - z0]);
end

figure
hold on
plot(r_left_enu(1, :), 'r')
plot(r_left_enu(2, :), 'g')
plot(r_left_enu(3, :), 'b')

plot(r_base_enu(1, :), 'r--')
plot(r_base_enu(2, :), 'g--')
plot(r_base_enu(3, :), 'b--')
