%% Iteratively solve to get the Tau at each position in workspace 
% figure out what the maximum tau values are for each joint check that they
% are within the correct lengths 

%% arbitrary values for now 


L = [30 35 35 10];
M = [0.4 0.4 0.4 0.5 0.2];
Tau = torque_required(Q,L,M);


%% get the range of joint angles 

q1_range = [(-deg2rad(135)):0.2:deg2rad(135)];
dq_1 = size(q1_range,2);

% dont want to rotate higher or lower than 90 degrees for second joint 
q2_range =[0:0.1:deg2rad(90)];
dq_2 = size(q2_range,2);

% elbow joint never want to rotate 'forearm' above the 'upper arm'
% also ensure arm is never straight - by not letting t3 = 0 degrees 
q3_range = [-deg2rad(135):0.1:-deg2rad(2)];
dq_3 = size(q3_range,2);

% initialise tau variables 
Max_Tau = [0 0 0 0]; 

for i = 1:dq_1
    q1 = q1_range(i) ;
    for j = 1:dq_2 
        q2 = q2_range(j);
        for k = 1:dq_3
            q3 = q3_range(k);
            % apply constraint on q4 to ensure end effector is pointing
            % down 
            q4 = -q3-q2; 
            
            Q = [q1 q2 q3 q4]; 
            % calculate the torque: 
            Tau = torque_required(Q,L,M);
            
            %update maximum torques 
            if abs(Tau(1)) > abs(Max_Tau(1))
                Max_Tau(1) = Tau(1);
            end 
            if abs(Tau(2)) > abs(Max_Tau(2))
                Max_Tau(2) = Tau(2);
            end 
            if abs(Tau(3)) > abs(Max_Tau(3))
                Max_Tau(3) = Tau(3);
            end 
            if abs(Tau(4)) > abs(Max_Tau(4))
                Max_Tau(4) = Tau(4);
            end 
        end 
    end 
end 

Max_Tau