function [ res ] = experiment(TLEnumber, seed, timestep, iterations_total, iterations_start, Q, P, R)

%% process args

bspFile = ['bsp/TLE',num2str(TLEnumber),'.bsp'];
cspice_furnsh(bspFile);

%% initial
initialState = getMeasuredState(0, bspFile);
initialTLEelements = rv2tleNEWCOR(initialState);
TLEelements = initialTLEelements;

TLEcovarianceSQRT = chol(P,'lower');
measCovarianceSQRT = chol(R,'lower');
processNoiseSQRT = chol(Q,'lower');
errorPos = zeros(1, iterations_total);

%% sim
for i = 1:iterations_total
    tsince = timestep * i;
    currentMeas = getMeasuredState(tsince, bspFile);
    [TLEelements, TLEcovarianceSQRT] = ...
    SRCKF4TLE(TLEelements, TLEcovarianceSQRT, currentMeas, measCovarianceSQRT, processNoiseSQRT, timestep);
    errorPos(i) = getCurrPosError(TLEelements, tsince, bspFile);
end

res = rms(errorPos(iterations_start:end));
cspice_unload(bspFile);
end

