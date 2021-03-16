clear 
close all
clc

addpath('classes');
addpath('tools');

dim = 2;
step_size = [8]*pi/180;
random_world = 0;
show_output = 1;
num_of_runs = 1;

for idx=1:size(step_size,1)
    segmentLength = step_size(idx);
    
    time = 0;
    avg_its = 0;
    avg_path = 0;
    for i=1:num_of_runs
        [n_its, path_n, run_time] =  RRTconnect(dim,segmentLength,random_world,show_output);
        time = time + run_time;
        avg_its = avg_its + n_its;
        avg_path = avg_path + path_n;            
    end
    
    str1 = ['The time taken by RRT-Connect for ', num2str(num_of_runs), ' runs is ', num2str(time)];
    str2 = ['The averagae time taken by RRT-Connect for each run is ', num2str(time/num_of_runs)];
    str3 = ['The averagae number of states explored by RRT-Connect for each run is ', num2str(avg_its/num_of_runs)];%random times
    str4 = ['The averagae number of state in Path by RRT-Connect for each run is ', num2str(avg_path/num_of_runs)];        
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
    disp(str1);
    disp(str2);
    disp(str3);
    disp(str4);
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
end



