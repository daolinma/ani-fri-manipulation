
figure(50)
hold on 
axis equal
radius_start = 0.015;


sim_filenames = dir('SimResult//*.mat');
mn_sim_filenames = size(sim_filenames);
for idx = 3:mn_sim_filenames
    load(['SimResult//',sim_filenames(idx).name])
    size_q = size(q);
    plot(radius_start*cos(q(3,1)) + q(1,1:5:size_q(2)), radius_start*sin(q(3,1)) + q(2,1:5:size_q(2)))
 


end