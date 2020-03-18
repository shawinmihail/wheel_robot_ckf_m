function [ vEnu ] = ned2enu( vNed )
vEnu = [vNed(2); vNed(1); -vNed(3)];
end

