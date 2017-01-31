close all
clear all

% directoryname = '/rect_push/rect1_json/'
directoryname = '/Repeated/plywood_rect1_json/'
filelist = dir([pwd,directoryname]);
 
mn = size(filelist)
offset_dir = 2;


orientation_obj_ini = zeros(mn(1),1);
orientation_obj_final = zeros(mn(1),1);
direction_tip = zeros(mn(1),1);

 figure(22) 
 hold on
 
 figure(23) 
 hold on
for i = 1+offset_dir:mn(1)
    filename = [pwd,directoryname,filelist(i).name]
    shape_id = 'rect1';
    do_plot = 0; 

    [object_pose, tip_pose, wrench] = get_and_plot_data(filename, shape_id, do_plot);
    
%     figure(2)
%     plot(tip_pose(:,2),tip_pose(:,3),'r')
%     hold on
%     plot(object_pose(:,2),object_pose(:,3))
%     hold off
%     title('position of center of mass in plane')
%     legend('Tip','Object')
%     
%     figure(3) 
%     plot(wrench(:,2),wrench(:,3))
    
% plot robot based cm positions
    figure(22) 
    plot(object_pose(:,2),object_pose(:,3))
    plot(object_pose(1:3,2),object_pose(1:3,3),'--gs','MarkerSize',10,'MarkerEdgeColor','r')
    
% calculate and plot object based cm positions
    pose_inobjectfram = [object_pose(:,2),object_pose(:,3)] * [cos(object_pose(1,4)), -sin(object_pose(1,4));sin(object_pose(1,4)), cos(object_pose(1,4))];
    figure(23) 
    plot(pose_inobjectfram(:,1) - pose_inobjectfram(1,1),pose_inobjectfram(:,2)-pose_inobjectfram(1,2))
    
% find the initial orientation of obj
    orientation_obj_ini(i) = (object_pose(1,4));
    
    size_this_push = size(object_pose);
% find the final orientation of obj
    orientation_obj_final(i) = object_pose(size_this_push(1),4);
    
    %calculate the direction of tip
    close(figure,1)

    i
end

% plot Trajectory of CM in Robot Base 
figure(22)
hold off
title('Trajectory of CM in Robot Base ')
xlabel('x (m)')
ylabel('y (m)')
axis equal
savefig('Trajectory of CM in Robot Base .fig')
saveas(gcf,'Trajectory of CM in Robot Base .eps','psc2')
saveas(gcf,'Trajectory of CM in Robot Base .png')
close(figure,22)

% plot Trajectory of CM in Object Base 
figure(23)
hold off
title('Trajectory of CM in Object Base ')
xlabel('x (m)')
ylabel('y (m)')
axis equal
savefig('Trajectory of CM in Object Base .fig')
saveas(gcf,'Trajectory of CM in Object Base .eps','psc2')
saveas(gcf,'Trajectory of CM in Object Base .png')
close(figure,23)

% plot munipulation angle/ Initial object orientaion
figure(3)
plot(orientation_obj_ini)
title('munipulation angle/ Initial object orientaion')
xlabel('num of push')
ylabel('angle')

% plot Final object orientaion
figure(13)
plot(orientation_obj_final)
title('Final object orientaion')
xlabel('num of push')
ylabel('angle')


diff_orientation = diff(orientation_obj_ini);
for i=1:mn(1)-1
    if diff_orientation(i) < -4
        diff_orientation(i) = diff_orientation(i) + pi*2;
    end
end

figure(4)
plot(diff_orientation*180/pi)    
    
accum_orientation_obj_ini = cumsum(diff_orientation);

% plot accumed orientaion of object in robot base
figure(5)
plot(accum_orientation_obj_ini*180/pi)    
title('Accumed Orientaion of Object in Robot Base ')
xlabel('Counts of Pushing (N)')
ylabel('Accumed Orientation (degree)')
savefig('Accumed Orientaion of Object in Robot Base.fig')
saveas(gcf,'Accumed Orientaion of Object in Robot Base.eps','psc2')
saveas(gcf,'Accumed Orientaion of Object in Robot Base.png')
