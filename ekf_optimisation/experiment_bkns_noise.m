function [ res ] = experiment_bkns_noise(TLEnumber, timestep, iterations_total, iterations_start, Q, P, R, alpha, noise_array)

%% process args

bspFile = ['bsp2/TLE',num2str(TLEnumber),'.bsp'];
cspice_furnsh(bspFile);

%% initial
initialState = getMeasuredState(0, bspFile);
initialTLEelements = rv2tleNEWCOR(initialState);
TLEelements = initialTLEelements;

sqrtP = chol(P,'lower');
sqrtR = chol(R,'lower');
sqrtQ = chol(Q,'lower');
%% sim
for i = 1:iterations_total
    tsince = timestep * i;
    rv_mes = get_measured_state_bkns(tsince, bspFile, noise_array);
%     r_mes = rv_mes(1:3);
%     v_mes = rv_mes(4:6);
    
    [TLEelements, sqrtP, sqrtQ, sqrtR] = SRCKF4TLE_QR_evolution(TLEelements, sqrtP, rv_mes, sqrtR, sqrtQ, alpha, timestep);
    
    satrec = elements2satrec(TLEelements);
    [r_est, v_est] = SGP4.SGP4(satrec, 0);
    
    rv_act = getActualState(tsince, bspFile);
    r_act = rv_act(1:3);
    v_act = rv_act(4:6);
    

    r_error_est(i) = 1000*norm(r_est-r_act);
    v_error_est(i) = 1000*norm(v_est-v_act);
%     r_error_mes(i) = 1000*norm(r_mes-r_mes);
%     v_error_mes(i) = 1000*norm(v_mes-v_mes);
    
%     tle(i,:) = TLEelements;
end

res_r = rms(r_error_est(iterations_start:end));
res_v = rms(v_error_est(iterations_start:end));
res = (res_r + res_v*700)/2;
cspice_unload(bspFile);
end

