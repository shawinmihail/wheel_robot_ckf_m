function [calib_procedure_finished, calib_procedure_runs, dr1, dr2,...
    calib_r_buffer, calib_v_buffer, calib_dr1_buffer, calib_dr2_buffer, ts_slaves_calib, tf_slaves_calib] = ...
    ...
    slaves_calib(t, measure, ...
    calib_r_buffer, calib_v_buffer, calib_dr1_buffer, calib_dr2_buffer, ...
    calib_procedure_finished, calib_procedure_runs, ts_slaves_calib, tf_slaves_calib,...
    calib_duration, calib_v_max, calib_v_rms_max)
% SLAVES_CALIB is for calib gnns slaves

dr1 = [];
dr2 = [];

if ~calib_procedure_runs
    ts_slaves_calib = t;
    tf_slaves_calib = ts_slaves_calib + calib_duration;
    calib_r_buffer = [];
    calib_v_buffer = [];
    calib_dr1_buffer = [];
    calib_dr2_buffer = [];
    calib_procedure_runs = 1;
    warning('slaves_calib: collect data')
    
else
    if t > tf_slaves_calib
        warning('slaves_calib: process data')
        calib_procedure_runs = 0;
        rms_v = rms(calib_v_buffer);
        rms_r = rms(calib_r_buffer);
        rms_dr1 = rms(calib_dr1_buffer);
        rms_dr2 = rms(calib_dr2_buffer);
        % NO SUCCESS POSTPROC ENDS
        % 1. rms_v > rms v max val
        if max(rms_v) > calib_v_rms_max             
            warning(sprintf("slaves_calib: rms_v %f > rms v max val %f, restart calib", rms_v, calib_v_rms_max))
            warning("restart calib")
            calib_procedure_finished = 0;
        elseif 0
            % etc. rms check
            calib_procedure_finished = 0;
        else
            dr1 = mean(calib_dr1_buffer);
            dr2 = mean(calib_dr2_buffer);
            calib_procedure_finished = 1;
            warning(sprintf("slaves_calib: finished"))
            return
        end
        
    else
        calib_r_buffer = [calib_r_buffer; measure(1,1:3)];
        calib_v_buffer = [calib_v_buffer; measure(1,4:6)];
        calib_dr1_buffer = [calib_dr1_buffer; measure(1,7:9)];
        calib_dr2_buffer = [calib_dr2_buffer; measure(1,10:12)];
        st_base = measure(1,13);
        st_slave1 = measure(1,14);
        st_slave2 = measure(1,15);

        % NO SUCCESS RUNTIME ENDS
        % 1. status < 4
        if min([st_base, st_slave1, st_slave2]) < 3.5 % treat status as double 
            calib_procedure_runs = 0;
            calib_procedure_finished = 0;
            warning('slaves_calib: nl status < 4')
            warning("restart calib")
        end
        %(stricly moutionless)
        % 2. vel > vel max val
        if norm(measure(1,4:6)) > calib_v_max
            calib_procedure_runs = 0;
            calib_procedure_finished = 0;
            warning(sprintf("vel %f > vel max val %f", norm(measure(1,4:6)), calib_v_max))
            warning("restart calib")
        end
        % 3. base dr > base dr max val
        % 4. slaves dr > slaves dr max val
        % 5. imu a > imu a max val
        % 5. imu w > imu w max val
    end
end
    
    
  
end

