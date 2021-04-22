
%% Initialise paramaters of robot
L = [0.030 0.035 0.035 0.010];
% M(1)= m_l1;
% M(2)= m_l2;
% M(3)= m_l3;
% M(4)= m_l4;
% M(5)= m_mot3;
% M(6)= m_mot4; 
% M(7)= m_chess; 
M = [0.11, 0.114, 0.11, 0.022, 0.86, 0.86, 0.7];

% Q = [0,0,0,0.5]
% vpa(torque_required(Q,L,M))

%% Initialise variables for plot
% set the joint restraints (estimating about 270 degrees rotation) 
% allow maximum base rotation 

% t1_range = [(-deg2rad(135)):0.2:deg2rad(135)];
% dt_1 = size(t1_range,2);

% dont want to rotate higher or lower than 90 degrees for second joint 
q2_range =[(-deg2rad(45)):.3:deg2rad(89)];
dq_2 = size(q2_range,2);

q3_range = [(-deg2rad(135)):.4:(-deg2rad(1))];
dq_3 = size(q3_range,2);


% Plot colour and font size
% set(gca,'FontSize', 24)
set(gcf,'color','w');

%Plot reachable workspace
% counter = 1;     
for j = 1:dq_2
    t2 = q2_range(j);
    for k = 1:dq_3
        t3 = q3_range(k);
%         counter = counter+1


        % Solve forward kinematics and apply constraint to ensure end 
        % effector is always pointing straight down      
        t4  = - t3 - t2;
        Q = [0,t2,t3,t4];
        Tau = vpa(torque_required(Q,L,M));

        % plot the arm in this configuration
        forward_kinematics(Q, "Print", L, 5, Tau);
%         drawnow;

        % extract xyz coordinates for end effector position
        %For xz plot
%         x = T0E(1,4);
%         z = T0E(3,4);
%         X(counter) = x;
%         Y(counter) = z;
% 
% 
%         counter = counter + 1; 
    end

end


% set(gcf,'color','w');
% %XZ square for chessboard (sizes in the respective axes)
% translation_in_x = 26.5;
% %chessboard
% rectangle('Position',[translation_in_x,0,41.3, 7.5],'FaceColor',[0 .5 .5],'EdgeColor','b',...
%     'LineWidth',3)
% %peices headroom
% rectangle('Position',[translation_in_x + 0.25,7.5,41.3-0.5, 12],'FaceColor',[0.5 0.5 0.5],'EdgeColor','b',...
%     'LineWidth',3)
% hold on
% 
% %scatter plot of taskspace at height of chessboard
% scatter(X,Y, 10);
% xlabel('x_0 Displacement (cm)','FontSize',14);
% ylabel('z_0 Displacement (cm)','FontSize',14);
% % zlabel('z_0 Displacement (cm)','FontSize',14)
% title('XZ Reachable Workspace. Set 2 parameters.','FontSize',15);
% 
% 
