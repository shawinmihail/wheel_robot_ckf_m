function [ qEnu ] = quatNed2Enu( qNed )
qTransfer = [cos(pi/2); [1;1;0]*sin(pi/2)];
qTransfer = qTransfer / norm(qTransfer);
qEnu = quatMultiply(qTransfer, qNed);
end

