set(gcf,'color','w');
%% plot joint space 

% set the joint restraints (estimating about 270 degrees rotation) 
% allow maximum base rotation 
% t1_range = [-deg2rad(135):0.05:deg2rad(135)];
% dt_1 = size(t1_range,2);

% dont want to rotate higher or lower than 90 degrees for second joint 
t2_range =[0:0.5:deg2rad(90)]; 
dt_2 = size(t2_range,2);

% elbow joint never want to rotate 'forearm' above the 'upper arm' 
t3_range = [-deg2rad(135):0.5:0];
dt_3 = size(t3_range,2);

% final joint wont need to rotate beyond 90 degrees

% t4_range = [-deg2rad(90):0.1:deg2rad(90)]
% dt_4 = size(t4_range,2);

% is there a way to ensure the end effector is in the right position? i.e 
% pointing straight down -- maybe we could just do the plot for the joint 
% above the end effector and subtract the end effector height from Z
% coordinate? 

% change this at home, add dimensions 
tolerance = 0.001;
chessboard_height = 0.02; 
X = zeros(0);
Y = zeros(0);


% figure
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

%XZ square for chessboard (sizes in the respective axes)
rectangle('Position',[translation_in_x,0,41.3, 7.5],'FaceColor',[0 .5 .5],'EdgeColor','b',...
    'LineWidth',3)
hold on

%scatter plot of taskspace at height of chessboard
scatter(X,Y, 20)
xlabel('x_0 Displacement (cm)','FontSize',14)
ylabel('z_0 Displacement (cm)','FontSize',14)
% zlabel('z_0 Displacement (cm)','FontSize',14)
title('XZ Reachable Workspace. Set 1 parameters.','FontSize',15) 

