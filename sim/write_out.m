clc

fileID = fopen('est_states.floats','w');
data = [timeline' est_states']';
size_of_data = size(data);
fwrite(fileID, size_of_data, 'int');
fwrite(fileID, data, 'float');
fclose(fileID);

fileID = fopen('act_states.floats','w');
data = [timeline' act_states']';
size_of_data = size(data);
fwrite(fileID, size_of_data, 'int');
fwrite(fileID, data, 'float');
fclose(fileID);

fileID = fopen('mes_states.floats','w');
data = [timeline' mes_states']';
size_of_data = size(data);
fwrite(fileID, size_of_data, 'int');
fwrite(fileID, data, 'float');
fclose(fileID);

done = 'done'