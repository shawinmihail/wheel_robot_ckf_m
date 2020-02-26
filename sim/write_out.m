% csvwrite('out/est_states.csv',[timeline est_states])
% csvwrite('out/mes_states.csv',[timeline mes_states])
% csvwrite('out/act_states.csv',[timeline act_states])

fileID = fopen('est_states.floats','w');
fwrite(fileID,[1;2;3],'float');
fclose(fileID);