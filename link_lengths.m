function [done_flag] = link_lengths(plot_axes)

% the optimal link lengths in cm: 
L = [30 35 35 10];


% set the joint restraints (estimating about 270 degrees rotation) 
% allow maximum base rotation 

t1_range = [(-deg2rad(135)):0.2:deg2rad(135)];
dt_1 = size(t1_range,2);

% dont want to rotate higher or lower than 90 degrees for second joint 
t2_range =[0:0.1:deg2rad(90)];
dt_2 = size(t2_range,2);

% elbow joint never want to rotate 'forearm' above the 'upper arm'
% also ensure arm is never straight - by not letting t3 = 0 degrees 
t3_range = [-deg2rad(135):0.1:-deg2rad(2)];
dt_3 = size(t3_range,2);

X = zeros(0);
Y = zeros(0);% figure

counter=1;

%% plot workspace space 
if plot_axes == "XZ"



    %XZ plotting
    for j = 1:dt_2
        t2 = t2_range(j);
        for k = 1:dt_3
            t3 = t3_range(k);

            
            % Solve forward kinematics and apply constraint to ensure end 
            % effector is always pointing straight down      
            t4  = - t3 - t2;
            Q = [0,t2,t3,t4];
            T0E = forward_kinematics(Q,'No Print',5); 

            % extract xyz coordinates for end effector position
            %For xz plot
            x = T0E(1,4);
            z = T0E(3,4);
            X(counter) = x;
            Y(counter) = z;


            counter = counter + 1; 
        end

    end


    set(gcf,'color','w');
    %XZ square for chessboard (sizes in the respective axes)
    translation_in_x = 26.5;
    %chessboard
    rectangle('Position',[translation_in_x,0,41.3, 7.5],'FaceColor',[0 .5 .5],'EdgeColor','b',...
        'LineWidth',3)
    %peices headroom
    rectangle('Position',[translation_in_x + 0.25,7.5,41.3-0.5, 12],'FaceColor',[0.5 0.5 0.5],'EdgeColor','b',...
        'LineWidth',3)
    hold on

    %scatter plot of taskspace at height of chessboard
    scatter(X,Y, 10);
    xlabel('x_0 Displacement (cm)','FontSize',14);
    ylabel('z_0 Displacement (cm)','FontSize',14);
    % zlabel('z_0 Displacement (cm)','FontSize',14)
    title('XZ Reachable Workspace. Set 2 parameters.','FontSize',15);


    done_flag = 1;


elseif plot_axes == "XY"
    
    %XY plotting
    for i=1:dt_1
         t1 = t1_range(i);
         for j = 1:dt_2
            t2 = t2_range(j);
            for k = 1:dt_3
                t3 = t3_range(k);

                


                % Solve forward kinematics and apply constraint to ensure end 
                % effector is always pointing straight down
                t4  = - t3 - t2;
                Q = [t1,t2,t3,t4];
                T0E = forward_kinematics(Q,'No Print',5); 


                % extract xyz coordinates for end effector position        
                %For xy plot
                x = T0E(1,4);
                y = T0E(2,4);

                X(counter) = x;
                Y(counter) = y;
                counter = counter + 1;

            end
        end
    end

    translation_in_x = 26.5;

    % XY square for chessboard (sizes in the respective axes)
    rectangle('Position',[translation_in_x,-39.8/2, 41.3,39.8],'FaceColor',[0 .5 .5],'EdgeColor','b',...
        'LineWidth',3)
    hold on

    %scatter plot of taskspace at height of chessboard
    scatter(X,Y, 10);
    xlabel('x_0 Displacement (cm)','FontSize',14);
    ylabel('y_0 Displacement (cm)','FontSize',14);
    title('XY Reachable Workspace. Set 2 parameters.','FontSize',15); 


    done_flag = 1;

    
else
    done_flag = 0
end

end
