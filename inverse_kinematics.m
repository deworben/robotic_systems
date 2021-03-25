function [Q] = inverse_kinematics(x,y,z)
    %argument is the x,y,z coordinates  of the end effector in frame 0
    %(E_0)
    
    %arm length parameters
    l0 = 0.05;
    l1 = 0.30; %m
    l2 = 0.30; %m
    lE = 0.1; %m (length of end effector tube)
     
    %the target x,y in frame 0 determine q0 (the rotation of the base)
    q0 = atan2(y,x);
    
    % transformation from {0} to {1}
    T_10 = [cos(q0) sin(q0) 0 0;
            -sin(q0) cos(q0)  0 0;
            0          0     1 -l0;
            0          0     0   1];
        
    %rotate by q0 so that the new x and z axes (x1,y1) define the plane in which
    %the robot arm sits. x1 points along the plane of the robot arm
    E_0= [1 0 0 x;
          0 1 0 y;
          0 0 1 z;
          0 0 0 1];
 
        
    E_1 = T_10 * E_0;  %coordinates of end effector in frame 1 (rotated base)
    
    % also need to translate up to the 2nd joint. The end effectors height 
    % is effectively reduced in relation to the 'new origin'
    %% now solve for q1,q2,q3 treating the problem as 2D since we have rotated to the plane of the arm

    %the end effector coordinates of interest in the frame of the plane
    %translated up to the first joint at height l10 
    x_E = E_1(1,4);
    z_E = E_1(3,4);
    
    %translate up by the length of the end effector to point C (see
    %diagram)
    x_C = x_E;
    z_C = z_E + lE

    %these parameters don't depend on the quadrant or whether elbow is up
    %or down
    D = acos((x_C^2 + z_C^2 - (l1^2 + l2^2))/(2*l1*l2)); %by cosine rule
    beta = atan2(z_C, x_C);
    
    
    %% Quadrant 1 solutions (we are not always in quadrant 1, as the end effector may be lower than the 2nd joint 
    %elbow up solution (we want this one)
    q2 = -D;
    
    % because q2 is always negative, and gamma is less than 180 degrees, 
    % set gamma to be be pi + q2 (since this will subtract the negative
    % component  
    gamma = pi + q2; %internal angle created by the two arms 
    alpha = asin(l2*sin(gamma)/sqrt(x_C^2 + z_C^2)); %by sine rule
    q1 = beta + alpha;
    q3 = -(q2 + q1);
    
%     %elbow down solution
%     q2 = D;
%     gamma = abs(pi - q2); %internal angle created by the two arms 
%     alpha = asin(l2*sin(gamma)/sqrt(x_C^2 + z_C^2)); %by sine rule
%     q1 = beta - alpha;
%     q3 = q2 - q1;
    
%     % Quadrant 2 solutions 
%     %elbow up
%     q2 = D;
%     gamma = abs(pi - q2); %internal angle created by the two arms 
%     alpha = asin(l2*sin(gamma)/sqrt(x_C^2 + z_C^2)); %by sine rule
%     q1 = beta - (pi - alpha);
%     q3 = q2-q1;
    
%     %elbow down
%     q2 = -D;
%     gamma = abs(pi - q2); %internal angle created by the two arms 
%     alpha = asin(l2*sin(gamma)/sqrt(x_C^2 + z_C^2)); %by sine rule
%     q1 = beta + (pi - alpha);
%     q3 = q2-q1;
%     

%% NOTES
%unsure if the q3 calculation works for the elbow down solution or either
%quadrant 2 solution

%% Return Q
Q = [q0,q1,q2,q3];
end
