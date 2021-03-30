%% Written by M.Trollip, B.De Worsop and N.Parry 
% The script has three sections, each section carries out a list of tasks 

% Section 1: Forward Kinematics
% Section 2: Link Lengths Determination 
% Section 3: Inverse Kinematics

%% Forward Kinematics 

% symbolic variables, q are joint angles, l are link lengths  
syms q1 q2 q3 q4 l1 l2 l3 lE 

End_Effector_Frame = 5;

% Task 1: produce transformation matrics of frame i with respect to i-1
%         and show final transformation matrix w.r.t inertial frame 

    % First: create the DH Table 
     
     % symbolically assign the variables 
     a0 = 0;
     a1 = 0;
     a2 = l2; 
     a3 = l3; 
     d1 = l1;
     dE = lE;
     
     t1 = q1;
     t2 = q2;
     t3 = q3;
     t4 = q4;
     
     DH_Table = [a0 0 d1 t1; a1 pi/2 0 t2; a2 0 0 t3; a3 0 0 t4;...
         0 pi/2 dE 0];

    % Second: iteratively call DH_Transformation to get the necessary 
    %         transformation matrices, stored in an array of 4x4 matrices 
    
    for i = 1 : size(DH_Table,1)
        % extract paramaters from DH_Table 
        a_iminus = DH_Table(i,1);
        alph_iminus = DH_Table(i,2);
        d_i = DH_Table(i,3);
        t_i = DH_Table(i,4); 
        % transformation i expressed in i-1
        Timinus_i(:,:,i) = DH_Transformation(a_iminus, alph_iminus, ...
            d_i, t_i); 
    end 
    
    % Third: multiply through the previously calculated transformation
    %        matrices to get the end effector position in frame {0} 
    
    % get the pose for each frame expressed in frame{0}. 
    % Store them in an array of 4x4 matrices
    
    % frame {1} expressed in frame {0} is 
    T0i(:,:,1) = Timinus_i(:,:,1); 
    % store this for the loop 
    previous = T0i(:,:,1); 
    
    % loop through to get the pose for each frame expressed in the inertial 
    % frame. The last frame will be the end effector
     
    for i = 2:End_Effector_Frame
        T0i(:,:,i) = previous*Timinus_i(:,:,i);
        previous = T0i(:,:,i);
    end 
    
    %The end effector in frame {0} is therefore 
    T0E = T0i(:,:,End_Effector_Frame); 
    
% Task 2: produce end-effector pose given trivial joint space displacement 
   
    % task 1 is automated using the function 'foward_kinematics.m' 
    % forward_kinematics takes 3 arguments: 
    % - the joint space Q, 
    % - a string to specify if you want to print a stick figure, 
    % - the frame number you wish to ouput (i.e for end effector, would
    % enter 5) 
    % NOTE: the link lengths are specified in forward_kinematics.m
    
    % The pose is calculated: 
    Q_zero_position = [0 0 0 0];
    T0E_zero_position = forward_kinematics(Q_zero_position,'Print',...
        End_Effector_Frame); 
    

%% Link Lengths

%Task 1: plot the range of points given by the end effector at a variety of
%joint angles. Iterate through these and create a plot that visualises both
%these points as well as what the chess board looks like
figure(3)
link_lengths("XZ");
figure(4)
%This next XY plot takes a long time (roughly 12 mins) before you get an xy plot
%worth using. It is commented out for convenience, however feel free to
%uncomment to verify. It is essentially the same code, just wrapped in
%another for loop and only visualising the XY instead of XZ axes
% link_lengths("XY");

%Task 2: Task 2 involves iterating the variables in the forward_kinematics
%script such that the right configuration to fit our task (a complete
%coverage of the chessboard) is found. These parameters have been set as
%the default to demonstrate the result of this experimentation process

%Task 3: See report for further clarification. In summary - having link lengths 
%too long means that they're too heavy and possible motor failure will
%ensue whereas too short means not enough board coverage.




%% Inverse Kinematics 
%Task: demonstrate the inverse kinematics solution is correct by selecting
%2 specific locations to place a given piece on the chessboard and
%calculating the joint space solutions associated with each of the locations.
%Use the link lengths determined in the previous section.

%specify the two test positions on the chess board in frame 0 (A and B), cm
testA = [30, 30, 3]; %this is at roughly the height of the chessboard, simulating grabbing a piece
testB = [20, 10, 10]; %this is 10cm in the air, simulating moving between pieces
EPSILON = 0.1; %for testing if the solutions are close enough

%run the inverse kinematics script to determine the corresponding joint
%space solutions (show elbow up and elbow down), then run the forward 
%kinematics script to verify that these joint space
%solutions result in the correct end effector pose (ie the specified test
%position). 

%test A
%elbow up solution
Q = inverse_kinematics(testA(1), testA(2), testA(3), 'up');

%elbow up verification
T_05 = forward_kinematics(Q, 'Print', 5);
if ((abs(T_05(1,4) - testA(1)) < EPSILON) && (abs(T_05(2,4) - testA(2)) < EPSILON) && (abs(T_05(3,4) - testA(3)) < EPSILON))
    fprintf('test A passed, elbow up configuration\n');
else 
    fprintf('test A failed, elbow up configuration\n');
end

%elbow down solution
Q = inverse_kinematics(testA(1), testA(2), testA(3), 'down');

%elbow down verification
T_05 = forward_kinematics(Q, 'no', 5);
if ((abs(T_05(1,4) - testA(1)) < EPSILON) && (abs(T_05(2,4) - testA(2)) < EPSILON) && (abs(T_05(3,4) - testA(3)) < EPSILON))
    fprintf('test A passed, elbow down configuration\n');
else 
    fprintf('test A failed, elbow down configuration\n');
end

%note: although both the solutions are valid, the elbow up solution should
%be selected because that is the configuration the robot should always
%operate in

% %test B
%elbow up solution
Q = inverse_kinematics(testB(1), testB(2), testB(3), 'up');

%elbow up verification
T_05 = forward_kinematics(Q, 'no', 5);
if ((abs(T_05(1,4) - testB(1)) < EPSILON) && (abs(T_05(2,4) - testB(2)) < EPSILON) && (abs(T_05(3,4) - testB(3)) < EPSILON))
    fprintf('test B passed, elbow up configuration\n');
else 
    fprintf('test B failed, elbow up configuration\n');
end

%elbow down solution
Q = inverse_kinematics(testB(1), testB(2), testB(3), 'down');

%elbow down verification
T_05 = forward_kinematics(Q, 'no', 5);
if ((abs(T_05(1,4) - testB(1)) < EPSILON) && (abs(T_05(2,4) - testB(2)) < EPSILON) && (abs(T_05(3,4) - testB(3)) < EPSILON))
    fprintf('test B passed, elbow down configuration\n');
else 
    fprintf('test B failed, elbow down configuration\n');
end     
