set(gcf,'color','w');
%% plot joint space 

% set the joint restraints (estimating about 270 degrees rotation) 
% allow maximum base rotation 

t1_range = [(-deg2rad(135)):0.2:deg2rad(135)];
dt_1 = size(t1_range,2)


% dont want to rotate higher or lower than 90 degrees for second joint 
t2_range =[0:0.1:deg2rad(90)];
dt_2 = size(t2_range,2)

% elbow joint never want to rotate 'forearm' above the 'upper arm' 
t3_range = [-deg2rad(135):0.1:0];
dt_3 = size(t3_range,2)


% change this at home, add dimensions 
tolerance = 0.001;
chessboard_height = 7.5; 
X = zeros(0);
Y = zeros(0);
<<<<<<< HEAD
% figure
%XZ plotting
counter = 1;
for j = 1:dt_2
    t2 = t2_range(j);
    for k = 1:dt_3
        t3 = t3_range(k);
        
        % ensure straight arm configuration doesnt occur
        if (t3 == t2) 
            continue 
        end 

%       OPTIONAL: Visualise one position of the end effector to test it's working
        for i=1:1
%         if rem(counter, 2) == 0
%             t2;
%             t3;
%             t4  = -t3 - t2;
%             Q = [0,t2,t3,t4];
%             forward_kinematics(Q,'Print',5);  
%         end
          end


        % Solve forward kinematics with end_effector @ 90 degrees
        % constraint
        t4  = - t3 - t2;
        Q = [0,t2,t3,t4];
        T0E = forward_kinematics(Q,'No Print',5); 

        % extract xyz coordinates for end effector position
        %For xz plot
        x = T0E(1,4);
        z = T0E(3,4);
        X(counter) = x;
        Y(counter) = z;
        
        %For xy plot
%         x = T0E(1,4);
%         y = T0E(2,4);
%         X(counter) = x;
%         Y(counter) = y;
%         
        counter = counter + 1;
        
        % OPTIONAL: store XY values if within correct chessboard height
        for i=1:1
        %                 if (z <=  chessboard_height + tolerance) ...
        %                         && (z >=  chessboard_height - tolerance)
        %                     X(end+1) = x
        %                     Y(end+1) = y  
        end    
    end
    
end

% %XY plotting
% X = zeros(0);
% Y = zeros(0);
% counter = 1;
% number_of_iterations=1;
% for i=1:dt_1
%      t1 = t1_range(i);
%      for j = 1:dt_2
%         t2 = t2_range(j);
%         for k = 1:dt_3
%             number_of_iterations = number_of_iterations+1
%             t3 = t3_range(k);
% 
%             % ensure straight arm configuration doesnt occur
%             if (t3 == t2) 
%                 continue 
%             end 
%         
%   
%             % Solve forward kinematics with end_effector @ 90 degrees
%             % constraint
%             t4  = - t3 - t2;
%             Q = [t1,t2,t3,t4];
%             T0E = forward_kinematics(Q,'No Print',5); 
% 
% 
%             % extract xyz coordinates for end effector position        
%             %For xy plot
%             x = T0E(1,4);
%             y = T0E(2,4);
%             z = T0E(3,4);
% 
%             %only plot pieces above the chessboard and less than 2x the height
%             %of a chess piece
% %             if (z<7.5 | z>7.5 + 0.5)
% % %                 a=1
% %                 continue
% %             end
% 
%             X(counter) = x;
%             Y(counter) = y;
%             counter = counter + 1
% 
%         end
%     end
% end


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

% XY square for chessboard (sizes in the respective axes)
% rectangle('Position',[translation_in_x,-39.8/2, 41.3,39.8],'FaceColor',[0 .5 .5],'EdgeColor','b',...
%     'LineWidth',3)
% hold on

%scatter plot of taskspace at height of chessboard
scatter(X,Y, 10)
xlabel('x_0 Displacement (cm)','FontSize',14)
ylabel('z_0 Displacement (cm)','FontSize',14)
% zlabel('z_0 Displacement (cm)','FontSize',14)
title('XZ Reachable Workspace. Set 2 parameters.','FontSize',15) 

