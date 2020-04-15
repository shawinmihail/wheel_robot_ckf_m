function [vx, vy, vz] = vel_from_pos(rx, ry, rz, t)

dt = diff(t);

vx = diff(rx) ./ dt;
vy = diff(ry) ./ dt;
vz = diff(rz) ./ dt;

vlim = 1.2;

vx(vx>vlim) = vlim;
vx(vx<-vlim) = -vlim;
vy(vy>vlim) = vlim;
vy(vy<-vlim) = -vlim;
vz(vz>vlim) = vlim;
vz(vz<-vlim) = -vlim;

vx = smooth(vx);
vy = smooth(vy);
vz = smooth(vz);

vx = [vx(1); vx];
vy = [vy(1); vy];
vz = [vz(1); vz];

end

