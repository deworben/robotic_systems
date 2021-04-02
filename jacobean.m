function [J] = jacobean(Q,L,frame)

% link lenghts 
l1 = L(1);
l2 = L(2);
l3 = L(3);
lE = L(4); 

% get the rotation matrices from i to 0 and the z_i axes in frame 0 
for i = 1:5
    % transformations from {i} to {0} are 
    T0i(:,:,i) = forward_kinematics(Q,'no print',i);
    % rotation matrices from {i} to {0} are:
    R0i(:,:,i) = T0i(1:3,1:3,i);
    % z_i axis in frame 0 is: 
    z_0i(:,i) = R0i(:,:,i)*[0;0;1];
end 

% length from joint i to COM of each link IN frame {0}
p_1_c1_0 = R0i(:,:,1)*[0; 0; l1/2];
p_2_c2_0 = R0i(:,:,2)*[l2/2; 0; 0];
p_3_c3_0 = R0i(:,:,3)*[l3/2; 0; 0]; 
p_4_c4_0 = R0i(:,:,4)*[0; lE/2; 0];

if frame == 5 
    % calculating the jacobean for the end effector 
    isCOM = 'false'; 

elseif frame == 4
    % calculating J for COM of link 4
    isCOM = 'true';
    L_joint_to_COM = p_4_c4_0; 
 

elseif frame == 3
    % calculating J for COM of link 3 
    isCOM = 'true';
    L_joint_to_COM = p_3_c3_0;  

elseif frame == 2
    % calculating J for COM of link 2
    isCOM = 'true';
    L_joint_to_COM = p_2_c2_0;  

elseif frame == 1
    % calculating J for COM of link 1 
    isCOM = 'true';
    L_joint_to_COM = p_1_c1_0; 

else
    fprintf('ERROR: enter frame number between 1 and 5') 
    return 
end 


    
if isCOM == "false"
    % get the displacement vectors from each frame to end effector
    for i = 1:frame-1
        %transfomration matrix expressing {f} in {i}
        TiF(:,:,i) = inv(T0i(:,:,i))*T0i(:,:,frame);
        
        diF(:,i) = TiF(1:3,4,i);
        %displacement from {i} to {f} in frame {0}
        diF(:,i) = T0i(1:3,1:3,i)*diF(:,i);
    end 


elseif isCOM == "true" 
    % else get the displacement vectors from origin to COM of link 
    for i = 1:frame 
        %transfomration matrix from {f} to {i}
        TiF(:,:,i) = inv(T0i(:,:,i))*T0i(:,:,frame);

        diF(:,i) = TiF(1:3,4,i);
        
        %displacement from {i} to {F} in frame {0}
        diF(:,i) = T0i(1:3,1:3,i)*diF(:,i);

        % add the extra length from the joint at F to the COM 
        diF(:,i) = diF(:,i) + L_joint_to_COM;
    end 
end   

% initliase the jacobean  
J = sym(zeros(6,4));

% populate the jacobean 
if isCOM == "false" 
    for i = 1:frame-1
        %translational component 
        top = simplify(symbolic_cross(z_0i(:,i),diF(:,i)));
        % rotational component 
        bottom = simplify(z_0i(:,i));
        J(:,i) = [top;bottom];
    end 
end 

if isCOM == "true" 
    for i = 1:frame
        %translational component 
        top = simplify(symbolic_cross(z_0i(:,i),diF(:,i)));
        % rotational component 
        bottom = simplify(z_0i(:,i));
        J(:,i) = [top;bottom];
    end 
end 
