function r = cam_mes_to_r(sectorIdx,dist)

M_PIf32 = 3.1415;
m_minPhi = -M_PIf32;
m_maxPhi = M_PIf32;
m_phiSegmentsNum = 288;
m_phiSegmentLength = (m_maxPhi - m_minPhi) / m_phiSegmentsNum;

m_minTheta = 0.0;
m_maxTheta = M_PIf32;
m_thetaSegmentsNum = 144;
m_thetaSegmentLength = (m_maxTheta - m_minTheta) / m_thetaSegmentsNum;

m_minDistance = 0.0;
m_maxDistance = 50.0;
m_minSectorLength = 0.1;
m_squareDistanceSectorLengthCoef = 0.05;

phiIdx = floor(mod(sectorIdx, m_phiSegmentsNum));
thetaIdx = floor(sectorIdx / m_phiSegmentsNum);

minPhi = m_minPhi + phiIdx*m_phiSegmentLength;
maxPhi = minPhi + m_phiSegmentLength;

minTheta = m_minTheta + thetaIdx*m_thetaSegmentLength;
maxTheta = minTheta + m_thetaSegmentLength;

theta = (maxTheta + minTheta) / 2;
phi = (maxPhi + maxPhi) / 2;

x = dist * sin(theta) * cos(phi);
z = dist * sin(theta) * sin(phi);
y = dist * cos(theta);

r = [x;y;z];

end

