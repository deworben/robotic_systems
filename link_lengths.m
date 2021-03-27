
%% Confirm forward Kinematics 

% Q = [0,0,0,0]
% T0E = forward_kinematics(Q,'Print')

%% plot joint space 

% set the joint restraints (estimating about 270 degrees rotation) 
% allow maximum base rotation 
t1_range = [-deg2rad(135):0.1:deg2rad(135)];
dt_1 = size(t1_range,2);

% dont want to rotate higher or lower than 90 degrees for second joint 
t2_range =[-deg2rad(90):0.5:deg2rad(90)]; 
dt_2 = size(t2_range,2);

% elbow joint never want to rotate 'forearm' above the 'upper arm' 
t3_range = [-deg2rad(135):0.5:0];
dt_3 = size(t3_range,2);

% final joint wont need to rotate beyond 90 degrees

t4_range = [-deg2rad(90):0.5:deg2rad(90)]; 
dt_4 = size(t4_range,2);

% is there a way to ensure the end effector is in the right position? i.e 
% pointing straight down -- maybe we could just do the plot for the joint 
% above the end effector and subtract the end effector height from Z
% coordinate? 

% change this at home, add dimensions 
tolerance = 0.001;
chessboard_height = 0.02; 
X = zeros(0);
Y = zeros(0);
% plot maximum and minimum reach of the end effector arm 
for i = 1:dt_1 
    q_1 = i;
    Q_max = [q_1,0,0,0]; 
    Q_min = [q_1,deg2rad(90),deg2rad(-180),-(deg2rad(90)+deg2rad(-90))];
    TE0_max = forward_kinematics(Q_max,'No Print',5);
    TE0_min = forward_kinematics(Q_min,'No Print',5);
    
    XMAX(i) = TE0_max(1,4);
    YMAX(i) = TE0_max(2,4);
    
    X_MIN(i) = TE0_min(1,4);
    Y_MIN(i) = TE0_min(2,4);
    
    
    
end

    plot(XMAX,YMAX,'o')
    hold on 
    plot(X_MIN,Y_MIN,'o')
    hold on 
hold off 

