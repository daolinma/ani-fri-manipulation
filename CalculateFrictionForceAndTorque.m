function [ Force_x, Force_y, Torque ] = CalculateFrictionForceAndTorque(x,y,theta,v_x,v_y,x_c,y_c,omega,a,b,M,N,mass,g,mu_a,mu_b)
%This function numerically returns the friction force and torque from the
%plane to the pushed object.
% (x,y) is the current position of CM in the world frame, beta is the orientation
% angle of the object relative to the world frame
% (v_x,v_y) is the current velocity of CM in the world frame
% omega is the current rotation angular speed of the object in the world
% a and b are the length and width of rectangle respectively
% M and N the number of discretization along a and b. 
% mass is the mass of the rectangle
% g is gravity acceleration
% mu_a and mu_b are the two eigen coefficient of friction along x and y directions on the supporting surface.  Supposing mu_a in x direction and mu_b in y direction
% mu_a,mu_b should be larger than 0.

%   Detailed explanation goes here

% mean presure
mean_F = mass*g/(M*N);
Force_x = 0;
Force_y = 0;
Torque = 0;
Rot = [cos(theta), sin(theta); -sin(theta),cos(theta)];

for i =1:M
    for j=1:N
%         omega_vec = [0,0,omega];
%         arm_vec = [x-x_c,y-y_c,0];
%         velocity = cross(omega_vec,arm_vec);

        x_local_in_obj = (i-1/2)/M*a;
        y_local_in_obj = (j-1/2)/N*b;
        pos_local_in_obj    = [x_local_in_obj,y_local_in_obj];
        pos_local_in_world  = pos_local_in_obj*Rot;
        
        vel_CM_in_world = [v_x,v_y];
        vel_local_in_world  = vel_CM_in_world + [-omega*(y_local_in_obj-b/2),omega*(x_local_in_obj-a/2)]*Rot;
%         vel_local_in_world  = [-omega*(y-y_c),omega*(x-x_c)];
%         v_x_local = -omega*(y-y_c);
%         v_y_local = omega*(x-x_c);
        
        
        [force_x,force_y] = GetAnisotropicFrictionForce(vel_local_in_world,mu_a,mu_b,mean_F)
        
        Force_x = Force_x + force_x;
        Force_y = Force_y + force_y;
        torque_local = (pos_local_in_world(1) - x)*force_y - (pos_local_in_world(2) - y)*force_x;
        Torque = Torque + torque_local;
    end
end

end

