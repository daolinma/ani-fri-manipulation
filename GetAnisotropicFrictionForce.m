function [force_x,force_y] = GetAnisotropicFrictionForce(velocity,mu_a,mu_b,NormalForce)
%GetAnisotropicFrictionForce will get a friction force given a velocity and
%the  long radius and short radius of anisotropic friction limit circle.
% Supposing mu_a in x direction and mu_b in y direction
% mu_a,mu_b should be larger than 0
% N is the normal force, larger than 0
%   Detailed explanation goes here
v_x = velocity(1); 
v_y = velocity(2); 
theta = atan2(mu_b*v_y,mu_a*v_x);

force_x = mu_a*cos(theta)*NormalForce;
force_y = mu_b*sin(theta)*NormalForce;

end

