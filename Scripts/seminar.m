% Fakultet elektrotehnike i računarstva u Zagrebu

% Projekt iz kolegija Seminar 2022./2023.

% Enio Krizman 0069083848

%% Matlab code for determining the working space of a 5R planar parallel robot

% units = mm
%% 

clear all;
close all;

%The distance between actuators ("d") that gives the maximum working space
max_i = 0;
solutions_ = [];
Binary = [];
d_optimal = 0;

for d = 300:1:600
    % Searching for the optimal d
    [sol_,B_, i] = get_workspace(d);
    if i>max_i
        max_i = i;
        solutions_ = sol_;
        Binary = B_;
        d_optimal = d;
    end
end


plot_all = 0;

if plot_all == 1
    for d = 100:100:900
        % Plotting every 50th d solution
        [sol_,B_, i] = get_workspace(d);
        x = sol_(:,1);
        y = sol_(:,2);
        plot_workspace(x,y,d)
    end
end

find_egg_shape_start = 0;

if find_egg_shape_start==1
    for d = 640:1:650
        % Plotting every 50th d solution
        [sol_,B_, i] = get_workspace(d);
        x = sol_(:,1);
        y = sol_(:,2);
        plot_workspace(x,y,d)
    end
end

max_i_egg = 0;
solutions_egg = [];
Binary_egg = [];
d_optimal_egg = 0;

for d = 647:1:900
    % Searching for the optimal d - egg shape
    [sol_,B_, i] = get_workspace(d);
    if i>max_i_egg
        max_i_egg = i;
        solutions_egg = sol_;
        Binary_egg = B_;
        d_optimal_egg = d;
    end
end

% Plotting optimal d
x = solutions_(:,1);
y = solutions_(:,2);

plot_workspace(x,y,d_optimal)

% Plotting optimal d - egg shape
x_egg = solutions_egg(:,1);
y_egg = solutions_egg(:,2);

plot_workspace(x_egg,y_egg,d_optimal_egg)

idx = boundary(x_egg,y_egg);
bx = x(idx) ; by = y(idx);
A = polyarea(bx,by) 

[max_x, Ix] = max(x_egg);
[min_x, Ix] = min(x_egg);
[max_y, Iy] = max(y_egg);
[min_y, Iy] = min(y_egg);

x_values = [min_x-d_optimal_egg/2,max_x-d_optimal_egg/2 ]
y_values = [min_y,max_y]
%% 
function [solutions,B, i] = get_workspace(d)
    % bar length
    l0 = 230;
    max_val = 2*l0;

    if d>(4*l0)
        disp('d je prevelik!')
        return
    end
    
    % Allowed positions and angles : solutions = [x,y,theta1,theta2]
    solutions = zeros(500,4);

    % Binary map of space
    binary = zeros(100);
    
    % Row counter: u = x
    u = 0;
    % Column counter: v = y
    v = 0;
    
    % Allowed positions counter - determins the optimal d
    i = 1;
    
    for x = 0:1:d
        % For each new row reset solumn counter
        v = 0;
        u = u+1;
        for y = -500:10:500

            v=v+1;

            d1= d-x;
            
            % Distance from the actuator to end effector
            l1 = sqrt((x^2)+(y^2));
            l2 = sqrt((d-x)^2+y^2);
            
            
            if l1>max_val  | l2>max_val 
                % physical limitation:
                % l1 and l2 must be less than 2*l0
                % - two articles are stretched the most
                binary(v,u) = 0;
                continue;
            end
            
            % Inverse kinematics solver
            [theta1, theta2] = inverse_kinematics(x,y,d,l0,"low");
    
            if any(isnan([theta1, theta2]), 'all') | any(~isreal([theta1, theta2])) 
                % Invalid solutions 
                binary(v,u) = 0;
            elseif (theta1 >= 0) & (theta1<=180) & (theta2 >= 0) & (theta2 <=180)
                % Valid solutions
                binary(v,u) = 1;
                solutions(i,1) = x;
                solutions(i,2) = y;
                solutions(i,3) = theta1;
                solutions(i,4) = theta2;
                % Increase valid solutions counter
                i=i+1;
            else
                binary(v,u) = 0;
            end     
    
        end
    end
B = flip(binary);
     
end

%% Inverse kinematics

function [theta1,theta2] = inverse_kinematics(x,y,d,l0, configuration)
             l1 = sqrt((x^2)+(y^2));
             l2 = sqrt((d-x)^2+y^2);

            % Angle between  line x=0 and l1
            beta1 = atand(y/x);
            beta2 = atand(y/(d-x));
            
            % Angle between lo and l1
            gamma1 = acosd((l1^2)/(2*l0*l1));
            gamma2 = acosd((l2^2)/(2*l0*l2));
            
           
            if configuration == "low"
                % Angle between line x=0 and l0
                alpha1 = gamma1-beta1;
                alpha2 = gamma2+beta2;
            elseif configuration == "up"
                % Angle between line x=0 and l0
                alpha1 = gamma1+beta1;
                alpha2 = gamma2-beta2;
            end

            % Inverse kinematics solution - actuator angles
            theta1 = 90 - alpha1;
            theta2 = 90 - alpha2;

end
%% Plot workspace

function [] = plot_workspace(x,y, d)
        
        x = x-d/2;
        figure()
        hold on
        n=numel(x);
        m = numel(y);
        plot( graph(1:n,1:m),'LineStyle','none','Marker','d','XData',x,'YData',y); 
        hold on
        plot(d/2,0,'k.', 'MarkerSize', 30);
        hold on
        plot(-d/2,0,'k.', 'MarkerSize', 30);
        title('Udaljenost između aktuatora d = ',num2str(d))
        hold off;
end


