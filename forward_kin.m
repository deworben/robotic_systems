%% Forward Kinematics: 
% derive forward kinematics using DH parameters 
% plotting the joint space of the robot to confirm that it can cover the
% chess board 

%% DH Table 
%syms d1 d2 d3 d4 dE a0 a1 a2 a3 de t1 t2 t3 t4 tE

t1 = 0;
t2 = 0; 
t3 = 0; 
t4 = 0;
Q = [t1,t2,t3,t4] ;

a0  = 0;
a1 = 0;
a2 = 30; 
a3 = 30; 
d1 = 10;
dE = 5; 

DH_Table = [a0 0 d1 t1; a1 pi/2 0 t2; a2 0 0 t3; a3 0 0 t4; 0 pi/2 dE 0];

%% Get the transformation matrices 

for i = 1 : size(DH_Table,1)
    
    % extract paramaters from DH_Table 
    a_iminus = DH_Table(i,1);
    alph_iminus = DH_Table(i,2);
    d_i = DH_Table(i,3);
    t_i = DH_Table(i,4); 
    
    % transformation i expressed in i-1
    Timinus_i(:,:,i) = DH_Transformation(a_iminus, alph_iminus, d_i, t_i); 

end 

%% Get the end transformation matrices to express the joints from origin 

T0i(:,:,1) = Timinus_i(:,:,1); 
previous = T0i(:,:,1); 

for i= 2:size(DH_Table) 
    T0i(:,:,i) = previous*Timinus_i(:,:,i)
    previous = T0i(:,:,i) 
end 

%% create 3 X,Y,Z Matrices to plot 
n_transforms = 5 
X = zeros(6);
Y = zeros(6);
Z = zeros(6); 
for i = 1:n_transforms
    X(i+1) = T0i(1,4,i);
    Y(i+1) = T0i(2,4,i);
    Z(i+1) = T0i(3,4,i);
end 

%% confirmation of forward kinematics  
Y(6,1) = 0;   % set this as was getting weird rounding error
plot3(X,Y,Z)
    

%% Determination of link lengths 

% set the joint restraints (estimating about 270 degrees rotation) 
theta_range = [-deg2rad(135):0.1:deg2rad(135)];


for i = 1:size(t1,2) 
    
    t1 = theta_range(i)
    
    for j = 1:size(t2,2)
        
        t2 = theta_range(j)
        
        for k = 1:size(t3,2)
            
            t3 = theta_range(k)
            
            for l = 1:size(t4,2)
                
                t4 = theta_range(l)
