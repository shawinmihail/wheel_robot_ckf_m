function [ res ] = experiment_rb(TLEnumber, timestep, iterations_total, iterations_start, Q, P, R, noise_array, bstar)

%% process args

bspFile = ['bsp2/TLE',num2str(TLEnumber),'.bsp'];
cspice_furnsh(bspFile);

%% initial
initialState = getMeasuredState(0, bspFile);
initialTLEelements = rv2tleNEWCOR(initialState);
TLEelements = initialTLEelements(1:6);

sqrtP = chol(P,'lower');
sqrtR = chol(R,'lower');
sqrtQ = chol(Q,'lower');
%% sim
for i = 1:iterations_total
    tsince = timestep * i;
    rv_mes = get_measured_state_bkns(tsince, bspFile, noise_array);
    r_mes = rv_mes(1:3);
    v_mes = rv_mes(4:6);

    [TLEelements, sqrtP, sqrtQ, sqrtR] = SRCKF4TLE_rb(TLEelements, sqrtP, rv_mes, sqrtR, sqrtQ, timestep, bstar);
    
    satrec = elements2satrec([TLEelements bstar]);
    [r_est, v_est] = SGP4.SGP4(satrec, 0);
    
    rv_act = getActualState(tsince, bspFile);
    r_act = rv_act(1:3);
    v_act = rv_act(4:6);
    

    r_error_est(i) = 1000*norm(r_est-r_act);
    v_error_est(i) = 1000*norm(v_est-v_act);
    r_error_mes(i) = 1000*norm(r_mes-r_act);
    v_error_mes(i) = 1000*norm(v_mes-v_act);
    
%     tle(i,:) = TLEelements;
end

rms_r_est = rms(r_error_est(iterations_start:end));
rms_v_est = rms(v_error_est(iterations_start:end));
rms_r_mes = rms(r_error_mes(iterations_start:end));
rms_v_mes = rms(v_error_mes(iterations_start:end));
% res = rms_v_est;
res = rms_r_est/2/rms_r_mes + rms_v_est/2/rms_v_mes;
cspice_unload(bspFile);
end

