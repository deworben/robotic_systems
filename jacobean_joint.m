function [J] = jacobean_joint(Q,L,frame)

% link lenghts 
l1 = L(1);
l2 = L(2);
l3 = L(3);
lE = L(4); 

% get the rotation matrices from i to 0 and the z_i axes in frame 0 
for i = 1:frame 
    % transformations from {i} to {0} are 
    T0i(:,:,i) = forward_kinematics(Q,'no print',L,i);
    % rotation matrices from {i} to {0} are:
    R0i(:,:,i) = T0i(1:3,1:3,i);
    % z_i axis in frame 0 is: 
    z_0i(:,i) = R0i(:,:,i)*[0;0;1];
end 


 % get the displacement vectors from each frame to joint 
 for i = 1:frame-1
     %transfomration matrix expressing {f} in {i}
     TiF(:,:,i) = (T0i(:,:,i))\T0i(:,:,frame);
     diF(:,i) = TiF(1:3,4,i);
     %displacement from {i} to {f} in frame {0}
     diF(:,i) = T0i(1:3,1:3,i)*diF(:,i);
 end 
 

% initliase the jacobean  
J = sym(zeros(6,4));

% populate the jacobean 
for i = 1:frame-1
        %translational component 
        top = symbolic_cross(z_0i(:,i),diF(:,i));
        % rotational component 
        bottom = z_0i(:,i);
        J(:,i) = [top;bottom];
end 
 
end 
