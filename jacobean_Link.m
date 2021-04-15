function [J] = jacobean_Link(Q,L,link_num)

% link lenghts 
l1 = L(1);
l2 = L(2);
l3 = L(3);
lE = L(4); 

% get the rotation matrices from i to 0 and the z_i axes in frame 0 
for i = 1:link_num
    % transformations from {i} to {0} are 
    T0i(:,:,i) = forward_kinematics(Q,'no print',L,i);
    % rotation matrices from {i} to {0} are:
    R0i(:,:,i) = T0i(1:3,1:3,i);
    % z_i axis in frame 0 is: 
    z_0i(:,i) = R0i(:,:,i)*[0;0;1];
end 


if link_num == 4 
    % calculating J for COM of link 4
    L_joint_to_COM = R0i(:,:,4)*[0; lE/2; 0];


elseif link_num == 3
    % calculating J for COM of link 3 
    L_joint_to_COM = R0i(:,:,3)*[l3/2; 0; 0]; 

elseif link_num == 2
    % calculating J for COM of link 2
    L_joint_to_COM = R0i(:,:,2)*[l2/2; 0; 0];  

elseif link_num == 1
    % calculating J for COM of link 1 
    L_joint_to_COM = R0i(:,:,1)*[0; 0; l1/2]; 

else
    fprintf('ERROR: enter link number between 1 and 4') 
    return 
end 


%% get relative displacement vectors   
 % else get the displacement vectors from origin to COM of link 
for i = 1:link_num
      %transfomration matrix from {f} to {i}
      TiF(:,:,i) = (T0i(:,:,i))\T0i(:,:,link_num);
      diF(:,i) = TiF(1:3,4,i);    
      %displacement from {i} to COM of joint in frame {0}
      diF(:,i) = T0i(1:3,1:3,i)*diF(:,i) + L_joint_to_COM;
end 
   

%% populate the jacobean 

% initliase the jacobean  
J = sym(zeros(6,4));

  for i = 1:link_num
     %translational component 
     top = symbolic_cross(z_0i(:,i),diF(:,i));
     % rotational component 
     bottom = z_0i(:,i);
     J(:,i) = [top;bottom];
  end  

