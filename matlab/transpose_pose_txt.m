clear; clc; close all;

%%
dir = '../tsdf_fusion/data/';
cd(dir)
save_dir = '/home/josh/kuka_automatic_localization/tsdf_fusion/data_new/';

for i = 0:10
    if i ~= 10
        file_name = ['frame-00000',num2str(i),'.pose.txt'];
    else
        file_name = ['frame-0000',num2str(i),'.pose.txt'];
    end
    
    M = dlmread(file_name);
    M = M(1:4,1:4)';
    M(1:3,4) = M(1:3,4)./1000;
    writematrix(M,[save_dir,file_name],'Delimiter',' ');
end
