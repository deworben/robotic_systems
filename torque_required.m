function [Tau] = torque_required(Q,L,M) 

%% The Jacobean for the end effector 
end_effector_frame = 5;

J_E = jacobean(Q,L,end_effector_frame); 

%% Get the jacobean for the COM of each link length 

J_1 = jacobean(Q,L,1);
J_2 = jacobean(Q,L,2);
J_3 = jacobean(Q,L,3);
J_4 = jacobean(Q,L,4);

Jv1 = J_1(1:3,:);
Jv2 = J_2(1:3,:);
Jv3 = J_3(1:3,:);
Jv4 = J_4(1:3,:);
JvE = J_E(1:3,:);
%% Get the torques 


m1 = M(1);
m2 = M(2);
m3 = M(3);
m4 = M(4);
m_chess = M(5); 
g=9.8;
% torque required 
Tau = Jv1.'*[0;0;-m1*g] + Jv2.'*[0;0;-m2*g] + Jv3.'*[0;0;-m3*g] + ...
    Jv4.'*[0;0;-m4*g] + JvE.'*[0;0;-m_chess*g];

