clc

% fileID = fopen('est_states.floats','w');
% data = [timeline' est_states']';
% size_of_data = size(data);
% fwrite(fileID, size_of_data, 'int');
% fwrite(fileID, data, 'float');
% fclose(fileID);
% 
% fileID = fopen('act_states.floats','w');
% data = [timeline' act_states']';
% size_of_data = size(data);
% size_of_data
% fwrite(fileID, size_of_data, 'int');
% fwrite(fileID, data, 'float');
% fclose(fileID);

fileID = fopen('mes_states.floats','w');
%                 0           1-3                4-6           7-9         
mes_states = [enu_timeline' r_mes_enu_list' v_mes_enu_list' dr1_mes_enu_list' dr2_mes_enu_list' imu_data_for_c_test']';
size_of_data = size(mes_states)
fwrite(fileID, size_of_data, 'int');
fwrite(fileID, mes_states, 'float');
fclose(fileID);

done = 'done'