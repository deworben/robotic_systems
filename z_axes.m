%% Get the z coordinates symboliccaly  
syms q1 q2 q3 q4 



Q = [q1 q2 q3 q4];


 
for i = 1:5
    % transformations from {i} to {0} are 
    T0i(:,:,i) = forward_kinematics(Q,'no',i);
    % z_i axis in frame 0 is: 
    z_0i(:,i) = T0i(1:3,1:3,i)*[0;0;1];
end 

for i = 1:4
    %transfomration matrix from {E} to {i}
    TiE(:,:,i) = inv(T0i(:,:,i))*T0i(:,:,5);
   
    diE(:,i) = TiE(1:3,4,i);
    %displacement from {i} to {E} in frame {0}
    diE(:,i) = T0i(1:3,1:3,i)*diE(:,i);
end 
% T01 = forward_kinematics(Q,'no',1);
% T02 = forward_kinematics(Q,'no',2);
% T03 = forward_kinematics(Q,'no',3);
% T04 = forward_kinematics(Q,'no',4);
% T0E = forward_kinematics(Q,'no',5);
% 
% z_01 = T01*[0;0;1;0];
% z_02 = T02*[0;0;1;0];
% z_03 = T03*[0;0;1;0];
% z_04 = T04*[0;0;1;0];
% z_0E = T0E*[0;0;1;0];
% 
% % transform from i to E
% T1E = inv(T01)*T0E;
% T2E = inv(T02)*T0E;
% T3E = inv(T03)*T0E;
% T4E = inv(T04)*T0E; 
% 
% %take the displacement 
% d1E = T1E(1:3,4);
% d2E = T2E(1:3,4);
% d3E = T3E(1:3,4);
% d4E = T4E(1:3,4);

% populate the jacobean 
% J = zeros(6,4);
% 
% get the translational component 
J = sym(zeros(6,4));
for i = 1:4
    %translational component 
    top = simplify(symbolic_cross(z_0i(:,i),diE(:,i)));
    bottom = simplify(z_0i(:,i));
    J(:,i) = [top;bottom];
end 




%% Work out the torques 

% extract the rotation matrices from {i} to {0}
for i = 1:5 
    R0i(:,:,i) = T0i(1:3,1:3,i);
end 

% for now, symbollicaly define the vecotrs l1,l2,l3,lE 
syms l1 l2 l3 lE

% the vectors from joint i to link i's COM IN frame {0} is: 
% look at zero position sketch for reference 
p_1_c1_0 = R0i(:,:,1)*[0; 0; l1/2];
p_2_c2_0 = R0i(:,:,2)*[l2/2; 0; 0];
p_3_c3_0 = R0i(:,:,3)*[l3/2; 0; 0]; 
p_4_c4_0 = R0i(:,:,4)*[0; lE/2; 0];



p_1_2_0 = 2*p_1_c1_0;
p_2_3_0 = 2*p_2_c2_0;
p_3_4_0 = 2*p_3_c3_0;
p_4_E_0 = 2*p_4_c4_0;

% for Jv1
z_01 = z_0i(:,1);

% for Jv2 
z_02 = z_0i(:,2);
p_1_c2_0 = p_1_2_0 + p_2_c2_0; 

% for Jv3
z_03 = z_0i(:,3);
p_1_c3_0 = p_1_2_0 + p_2_3_0 + p_3_c3_0; 
p_2_c3_0 = p_2_3_0 + p_3_c3_0;

% for Jv4 
z_04 = z_0i(:,4);
p_1_c4_0 = p_1_2_0 + p_2_3_0 + p_3_4_0 + p_4_c4_0;
p_2_c4_0 = p_2_3_0 + p_3_4_0 + p_4_c4_0;
p_3_c4_0 = p_3_4_0 + p_4_c4_0;

Jv1 = [symbolic_cross(z_01,p_1_c1_0) zeros(3,3)];

Jv2 = [symbolic_cross(z_01,p_1_c2_0) symbolic_cross(z_02,p_2_c2_0) zeros(3,2)];

Jv3 = [symbolic_cross(z_01,p_1_c3_0) symbolic_cross(z_02,p_2_c3_0) symbolic_cross(z_03,p_3_c3_0) zeros(3,1)];

Jv4 = [symbolic_cross(z_01,p_1_c4_0) symbolic_cross(z_02,p_2_c4_0) symbolic_cross(z_03,p_3_c4_0) symbolic_cross(z_04,p_4_c4_0)];

% the masses for each link: 
syms m1 m2 m3 m4 
g = 9.8 %m/s^2

% The torques due to gravity is therefore: 

G = Jv1.'*[0;0;-m1*g] + Jv2.'*[0;0;-m2*g] + Jv3.'*[0;0;-m3*g] + Jv4.'*[0;0;-m4*g];

% % get the vectors from the orgin to each link's COM in frame {0}
% % "current" is the vector from the origin to the most recent joint
% current = zeros(3,1); 
% for i = 1:4
%    p_1_ci_0(:,i) = current + p_i_ci_0(:,i);
%    %update current  
%    current = p_1_ci_0(:,i) + p_i_ci_0(:,i);
% end   
%  

% for i = 1:4
%     disp(p_1_ci_0(:,i))
% end 

% obtain the jacobean pertaining to each link COM position. 
% (note: there are 4 links, with 4th link being the end effector)
% note: gravity only contributes a linear force, therefore only need Jv 
% see Wrench Lecture, page 22/24 for reference 


% get displacement from frame {i} to {4} in {0}
