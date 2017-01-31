%% About: Daolin Ma write this code to simulate the dynamics of a pushed rectangle object on a plane with anisotropic friction
close all
clear all

%% parameters
a = 0.0450;             % length
b = 0.0450;             % width
h = 0.013;              % height &&&&&&&&&&&&&&&&&&&&&&&&&& Ask peter
m = 7900*a*b*h;         % mass &&&&&&&&&&&&&&&&&&&&&&&&&& Ask peter
radius_p = 0.075;       % pusher's radius

mu_a = 0.24;    % the eigen friction coeficient in x direction
mu_b = 0.28;    % the eigen friction coeficient in y direction
mu_p = 0.2;   % pushing
mu_p_static = mu_p*1.4;   % pushing

v_push =0.02;
% v_px = 0.02;
% v_py = 0;
m_max = 0.01;

epsilon_v = 1E-6;
epsilon_dis = 1E-6;
CONTACT = 1;
NONCONTACT = 0;
STATIC = 1;
SLIDING = 0;
Y_PLUS = 1;
Y_MINUS = -1;

M = 200;        % discretization in x direction of object
N = 200;        % discretization in y direction of object
t_start = 0;    % start time of simulation
t_end = 4;      % end time of simulation
time_step = 2.5E-3; % time step of simulation


for theta_0=0:0.1:pi
    %% initial conditions
    % initial position/orientation
    x_0 = 0;
    y_0 = 0;
    
    v_px = v_push*cos(theta_0);
    v_py = v_push*sin(theta_0);
    % theta_0 = 0;
    
    % initial velocity/angular velocity should be calculated by constrains.
    v_x_0 = 0;      %% supposing we are simulating the dynamic of pushed object in a framework of event-driven approach.
    v_y_0 = 0;
    omega_0 = 0;    % initial rotation angular velocity
    
    %%  contact between pusher and object
    k = 1E8;        % initial guess
    
    %% Dynamics of the rect
    
    t = t_start
    x = x_0;
    y = y_0;
    theta = theta_0;
    
    x_dot = v_x_0;
    y_dot = v_y_0;
    theta_dot = omega_0;
    
    
    idx =1; %counting index of iteration
    q(:,idx) = [x,y,theta];
    q_dot(:,idx) = [x_dot,y_dot,theta_dot];
    Force_seq(:,idx) = [0,0,0];
    %% Pushing motion calculation
    % function [val_s_vx, val_s_vy, val_s_omega, val_s_fx, val_s_fy, val_s_m] = CALCULATEMOTION(theta,v_px,v_py,x_c,y_c,m_max,mu_a,mu_b)
    syms s_x    s_y     s_theta     s_vx    s_vy    s_omega
    syms s_xc   s_yc    s_vpx   s_vpy
    syms s_fx   s_fy    s_m
    syms s_mmax s_fa    s_fb    s_mu_push
    
    % euations for static
    vec_normal = [1,0]*[cos(s_theta),-sin(s_theta);sin(s_theta),cos(s_theta)];
    % vec_tang = [0,1]*[cos(s_theta),-sin(s_theta);sin(s_theta),cos(s_theta)];
    s_v_tau = (s_vx - s_omega*s_yc - s_vpx)*(-sin(s_theta)) + (s_vy + s_omega*s_xc -s_vpy)*cos(s_theta);
    s_f_tau = (-s_fx)*(-sin(s_theta)) + (-s_fy)*cos(s_theta);
    s_f_n = (-s_fx)*(cos(s_theta)) + (-s_fy)*sin(s_theta);
    %  for static
    eq1 = (s_fx/s_fa)^2 + (s_fy/s_fb)^2 + (s_m/s_mmax)^2 -1;
    eq2 = s_vx*s_m - s_fx * s_omega * (s_mmax/s_fa)^2;
    eq3 = s_vy*s_m - s_fy * s_omega * (s_mmax/s_fb)^2;
    eq4 = s_m - s_xc * s_fy + s_yc * s_fx;
    eq7 = s_vx - s_omega*s_yc -s_vpx;
    eq8 = s_vy + s_omega*s_xc -s_vpy;
%     eq10= s_vx^2 + s_vy^2 + s_omega^2 -1;
    %  for sliding
    eq5 = (s_vx- s_omega*s_yc - s_vpx)*cos(s_theta) + (s_vy + s_omega*s_xc -s_vpy)*sin(s_theta);
    eq9 = (-(-s_fx)*sin(s_theta) + (-s_fy)*cos(s_theta) )^2 -  s_mu_push^2*((-s_fx)*cos(s_theta) + (-s_fy)*sin(s_theta))^2;
    % % solve nonlinear constraints for STATIC PUSHING
    [s_vx,s_vy,s_omega,s_fx,s_fy,s_m]=solve(eq2,eq3,eq4,eq7,eq8,eq1,'s_vx,s_vy,s_omega,s_fx,s_fy,s_m');
    simplify(s_vx);
    simplify(s_vy);
    simplify(s_omega);
    simplify(s_fx);
    simplify(s_fy);
    simplify(s_m);
    % solve nonlinear constraints for SLIDING PUSHING
    [s_vx_2,s_vy_2,s_omega_2,s_fx_2,s_fy_2,s_m_2]=solve(eq2,eq3,eq4,eq5,eq9,eq1,'s_vx,s_vy,s_omega,s_fx,s_fy,s_m');
    simplify(s_vx_2);
    simplify(s_vy_2);
    simplify(s_omega_2);
    simplify(s_fx_2);
    simplify(s_fy_2);
    simplify(s_m_2);
    % s_vx_2 =
    
    
    friction_state_pushing = STATIC;
    previou_friction_state_pushing = STATIC;
    x_p = -a/2 * cos(theta) - b*0.2* sin(theta);
    y_p = a/2 * sin(theta) - b * 0.2 * cos(theta);
    
    
    while t<= t_end
        TrueIdx = -1;
        % contact position
        
        direction_n = [cos(theta), sin(theta)];
        direction_tau = [-sin(theta), cos(theta)];
        
        distance_normal = [x-x_p,y-y_p]*direction_n';
        distance_tau = [x-x_p,y-y_p]*direction_tau';
%         if (distance_normal - a/2) < epsilon_dis && abs(distance_tau) < b/2
        if  abs(distance_tau) < b/2
            contact_state = CONTACT;
        else
            contact_state = NONCONTACT;
        end
        
        if contact_state == CONTACT
            x_c = x_p-x;
            y_c = y_p-y;
            % evaluate static
            if friction_state_pushing == STATIC
                val_s_vx =          subs(s_vx,{s_vpx,s_vpy,s_xc,s_yc,s_theta,s_mmax,s_fa,s_fb,s_mu_push},{v_px,v_py,x_c,y_c,theta,m_max,mu_a,mu_b,mu_p});
                val_s_vx = eval(val_s_vx);
                
                val_s_vy =          subs(s_vy,{s_vpx,s_vpy,s_xc,s_yc,s_theta,s_mmax,s_fa,s_fb,s_mu_push},{v_px,v_py,x_c,y_c,theta,m_max,mu_a,mu_b,mu_p});
                val_s_vy = eval(val_s_vy);
                
                val_s_omega =    subs(s_omega,{s_vpx,s_vpy,s_xc,s_yc,s_theta,s_mmax,s_fa,s_fb,s_mu_push},{v_px,v_py,x_c,y_c,theta,m_max,mu_a,mu_b,mu_p});
                val_s_omega = eval(val_s_omega);
                
                val_s_fx =          subs(s_fx,{s_vpx,s_vpy,s_xc,s_yc,s_theta,s_mmax,s_fa,s_fb,s_mu_push},{v_px,v_py,x_c,y_c,theta,m_max,mu_a,mu_b,mu_p});
                val_s_fx = eval(val_s_fx);
                
                val_s_fy =          subs(s_fy,{s_vpx,s_vpy,s_xc,s_yc,s_theta,s_mmax,s_fa,s_fb,s_mu_push},{v_px,v_py,x_c,y_c,theta,m_max,mu_a,mu_b,mu_p});
                val_s_fy = eval(val_s_fy);
                
                val_s_m =            subs(s_m,{s_vpx,s_vpy,s_xc,s_yc,s_theta,s_mmax,s_fa,s_fb,s_mu_push},{v_px,v_py,x_c,y_c,theta,m_max,mu_a,mu_b,mu_p});
                val_s_m = eval(val_s_m);
                % find the right one
                A = val_s_fx*val_s_vx'; 
                B = val_s_fy*val_s_vy';
                C = val_s_omega*val_s_m';
                val_s_f_tau = (-val_s_fx)*(-sin(theta)) + (-val_s_fy)*cos(theta);
                val_s_f_n = (-val_s_fx)*(cos(theta)) + (-val_s_fy)*sin(theta);
                
                for i=1:4
                    if A(i,i) + B(i,i) < 0  && C(i,i) < 0 && val_s_f_n(i) > 0
                        TrueIdx = i
                    end
                end
                
               
                
                if   abs(val_s_f_tau(TrueIdx)/ val_s_f_n(TrueIdx)) >  mu_p_static
                    friction_state_pushing = SLIDING;
                    previou_friction_state_pushing = STATIC;
                    if val_s_f_tau(TrueIdx)/ val_s_f_n(TrueIdx) > 0
                        current_pushing_friction_direction = Y_PLUS;
                    else 
                        current_pushing_friction_direction = Y_MINUS;
                    end
                    previous_pushing_friction_direction = current_pushing_friction_direction;
                end
          
                
                
            else if friction_state_pushing == SLIDING
                    val_s_vx =      subs(s_vx_2,{s_vpx,s_vpy,s_xc,s_yc,s_theta,s_mmax,s_fa,s_fb,s_mu_push},{v_px,v_py,x_c,y_c,theta,m_max,mu_a,mu_b,mu_p});
                    val_s_vx = eval(val_s_vx);
                    
                    val_s_vy =      subs(s_vy_2,{s_vpx,s_vpy,s_xc,s_yc,s_theta,s_mmax,s_fa,s_fb,s_mu_push},{v_px,v_py,x_c,y_c,theta,m_max,mu_a,mu_b,mu_p});
                    val_s_vy = eval(val_s_vy);
                    
                    val_s_omega = subs(s_omega_2,{s_vpx,s_vpy,s_xc,s_yc,s_theta,s_mmax,s_fa,s_fb,s_mu_push},{v_px,v_py,x_c,y_c,theta,m_max,mu_a,mu_b,mu_p});
                    val_s_omega = eval(val_s_omega);
                    
                    val_s_fx =       subs(s_fx_2,{s_vpx,s_vpy,s_xc,s_yc,s_theta,s_mmax,s_fa,s_fb,s_mu_push},{v_px,v_py,x_c,y_c,theta,m_max,mu_a,mu_b,mu_p});
                    val_s_fx = eval(val_s_fx);
                    
                    val_s_fy =       subs(s_fy_2,{s_vpx,s_vpy,s_xc,s_yc,s_theta,s_mmax,s_fa,s_fb,s_mu_push},{v_px,v_py,x_c,y_c,theta,m_max,mu_a,mu_b,mu_p});
                    val_s_fy = eval(val_s_fy);
                    
                    val_s_m =         subs(s_m_2,{s_vpx,s_vpy,s_xc,s_yc,s_theta,s_mmax,s_fa,s_fb,s_mu_push},{v_px,v_py,x_c,y_c,theta,m_max,mu_a,mu_b,mu_p});
                    val_s_m = eval(val_s_m);
                    
                    
                    val_s_v_tau = (val_s_vx - val_s_omega*y_c - v_px)*(-sin(theta)) + (val_s_vy + val_s_omega*x_c -v_py)*cos(theta);  % velocity of point on object versus pushing point
                    val_s_f_tau = (-val_s_fx)*(-sin(theta)) + (-val_s_fy)*cos(theta);       % friction force the the pusher act on the object
                    val_s_f_n = (-val_s_fx)*(cos(theta)) + (-val_s_fy)*sin(theta);
                    
                    % find the right one
                    A = val_s_fx*val_s_vx';
                    B = val_s_fy*val_s_vy';
                    C = val_s_omega*val_s_m';
                    D = val_s_v_tau * val_s_f_tau';
                    
%                     previous_pushing_friction_direction = current_pushing_friction_direction;
%                     if val_s_f_tau(TrueIdx)/ val_s_f_n(TrueIdx) > 0
%                         current_pushing_friction_direction = Y_PLUS;
%                     else 
%                         current_pushing_friction_direction = Y_MINUS;
%                     end
                    
                    for i=1:4
                        if A(i,i) + B(i,i) < 0 && C(i,i) < 0 && D(i,i) < 0 && val_s_f_n(i) > 0
                            if previou_friction_state_pushing == STATIC && previous_pushing_friction_direction * val_s_f_tau(i) < 0
                                break;
                            end
                            TrueIdx = i
                        end
                    end
                    previou_friction_state_pushing = friction_state_pushing;
                    %        need upgrade. for the switch sign
                    if  abs(val_s_f_tau(TrueIdx)) < epsilon_v
                        friction_state_pushing == STATIC
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                end
            end
        else
            break;
        end
        
        
        
        
        x = x + val_s_vx(TrueIdx)*time_step;  %integration
        y = y + val_s_vy(TrueIdx)*time_step;  %integration
        theta = theta + val_s_omega(TrueIdx)*time_step;  %integration
        x_p = x_p + v_px * time_step;
        y_p = y_p + v_py * time_step;
        
        idx = idx + 1;
        q(:,idx) = [x,y,theta]';
        q_dot(:,idx) = [val_s_vx(TrueIdx),val_s_vy(TrueIdx),val_s_omega(TrueIdx)]';
        Force_seq(:,idx) = [val_s_fx(TrueIdx),val_s_fy(TrueIdx),val_s_m(TrueIdx)]';
        % q_dot(:,idx) = [val_s_vx(TrueIdx),val_s_vy(TrueIdx),val_s_omega(TrueIdx)]';
        t = t+ time_step
    end
     
    save(['SimResult//mu_a=',num2str(mu_a),'mu_b=',num2str(mu_b),'mu_a=',num2str(mu_a),'m_max=',num2str(m_max),'theta_0=',num2str(theta_0),'.mat'],'q','q_dot')
     
end
    %% time integration using event driven approach
    % while t<= t_end
    %
    % %     F_push_normal = CalculatePushNormalForceSpring(x,y,theta,v_x,v_y,x_c,y_c,theta_dot,a,b,M,N,mass,g,mu_a,mu_b);
    %
    %     x_c = x - y_dot/theta_dot;
    %     y_c = y + x_dot/theta_dot;
    %     [Force_x, Force_y, Torque] = CalculateFrictionForceAndTorque(x,y,theta,v_x,v_y,x_c,y_c,theta_dot,a,b,M,N,mass,g,mu_a,mu_b);
    %
    %
    % %     x_dot = x_dot + (Force_x + F_push_x)/mass*time_step;  %integration
    % %     y_dot = y_dot + (Force_y + F_push_y)/mass*time_step;  %integration
    % %     theta_dot = theta_dot + (Torque + Torque_push)/mass*time_step;  %integration
    %
    %     idx = idx + 1;
    %     q(:,idx) = [x,y,theta];
    %     q_dot(:,idx) = [x_dot,y_dot,theta_dot];
    % end
    %
    % %% function Calculating Pushing Force And Torque
    % function  Force =  CalculatePushNormalForceSpring(x,y,theta,k,x_p,y_p,radius_p)
    %
    % e_1 = [cos(theta),sin(theta)];
    % vec_p2o = [x-x_p,y-y_p];
    %
    % delta = vec_p2o * e_1 - a/2 - radius_p;
    %
    % if delta > 0
    %     Force = 0;
    % else
    %     Force = k*abs(delta)^1.5;        %assuming elastic contact with Hertzian model
    % end
    %
    % end
    
    
    
