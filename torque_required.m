function [Tau] = torque_required(Q,L,M) 

%% The Jacobean for the end effector 
end_effector_frame = 5;

J_E = jacobean_joint(Q,L,end_effector_frame); 

%% Get the jacobean for the COM of each link length 

J_l1 = jacobean_Link(Q,L,1);
J_l2 = jacobean_Link(Q,L,2);
J_l3 = jacobean_Link(Q,L,3);
J_l4 = jacobean_Link(Q,L,4);

%% Get jacobean for motors 3 and 4 (note motor 2,1 will not contribute to torque reguired to hold structure up) 
 
J_m3 = jacobean_joint(Q,L,3);
J_m4 = jacobean_joint(Q,L,4); 


% take the translational component of jacobean 
Jvl1 = J_l1(1:3,:);
Jvl2 = J_l2(1:3,:);
Jvl3 = J_l3(1:3,:);
Jvl4 = J_l4(1:3,:);

Jvm3 = J_m3(1:3,:);
Jvm4 = J_m4(1:3,:); 

JvE = J_E(1:3,:);
%% Get the torques 

m_l1 = M(1);
m_l2 = M(2);
m_l3 = M(3);
m_l4 = M(4);
m_mot3 = M(5);
m_mot4 = M(6); 
m_chess = M(7); 
g=9.8;

% torque required 
Tau = Jvl1.'*[0;0;-m_l1*g] + Jvl2.'*[0;0;-m_l2*g] + Jvl3.'*[0;0;-m_l3*g]...
    + Jvl4.'*[0;0;-m_l4*g] + Jvm3.'*[0;0;-m_mot3*g]...
    + Jvm4.'*[0;0;-m_mot4*g] + JvE.'*[0;0;-m_chess*g];
    

