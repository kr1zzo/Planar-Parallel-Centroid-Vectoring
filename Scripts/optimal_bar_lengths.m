% Fakultet elektrotehnike i računarstva u Zagrebu

% Projekt iz kolegija Seminar 2022./2023.

% Enio Krizman 0069083848

%% Matlab code for determining the working space of a 5R planar parallel robot

% units = mm
%% prva duljina

clear all;
close all;

%The distance between actuators ("d") that gives the maximum working space
max_i = 0;
solutions_ = [];
Binary = [];
d_optimal = 0;
optimal_ratio = 0;

for d = 100:10:800
    for n = 1:1:10
        % Searching for the optimal d
        [sol_,B_, i] = get_workspace(d, 1/n, "first");
        %disp(i)
        if i>max_i
            max_i = i;
            solutions_ = sol_;
            Binary = B_;
            optimal_ratio = 1/n;
            d_optimal = d;
        end
    end
end


% Plotting optimal d
x = solutions_(:,1);
y = solutions_(:,2);

idx = boundary(x,y);
bx = x(idx) ; by = y(idx);
A1 = polyarea(bx,by) 

plot_workspace(x,y,optimal_ratio, d_optimal)
%% druga duljina

%The distance between actuators ("d") that gives the maximum working space
max_i = 0;
solutions_ = [];
Binary = [];
d_optimal = 0;
optimal_ratio = 0;

for d = 100:10:800
    for n = 1:1:10
        % Searching for the optimal d
        [sol_,B_, i] = get_workspace(d, 1/n, "second");
        %disp(i)
        if i>max_i
            max_i = i;
            solutions_ = sol_;
            Binary = B_;
            optimal_ratio = 1/n;
            d_optimal = d;
        end
    end
end


% Plotting optimal d
x = solutions_(:,1);
y = solutions_(:,2);

idx = boundary(x,y);
bx = x(idx) ; by = y(idx);
A2 = polyarea(bx,by) 

plot_workspace(x,y,optimal_ratio, d_optimal)
%% 
function [solutions,B, i] = get_workspace(d, ratio, longer)
    % bar length
    l0 = 230;
    %pretposavka
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
            [theta1, theta2] = inverse_kinematics(x,y,d,l0,"up",longer,ratio);
    
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

function [theta1,theta2] = inverse_kinematics(x,y,d,l0, configuration, longer, ratio)
             l1 = sqrt((x^2)+(y^2));
             l2 = sqrt((d-x)^2+y^2);
            
            if longer == "first"
                l01 = l0; l02= l0;
                l03 = l0*ratio; l04 = l0*ratio;
            elseif longer == "second"
                l01 = l0*ratio; l02= l0*ratio;
                l03 = l0; l04 = l0;
            end
            % Angle between  line x=0 and l1
            beta1 = atand(y/x);
            beta2 = atand(y/(d-x));
            
            % Angle between lo and l1
            gamma1 = acosd((l1^2+l01^2-l02^2)/(2*l01*l1));
            gamma2 = acosd((l2^2+l04^2-l03^2)/(2*l04*l2));
            
           
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

function [] = plot_workspace(x,y, ratio,d)
        % bar length
        l0 = 230;
        %pretposavka
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
        title(['d = ', num2str(d),' ratio = ',num2str(ratio)])
        hold off;
end


