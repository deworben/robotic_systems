%% Symbolic variables 
% joint angles 
syms q1 q2 q3 q4
Q = [q1 q2 q3 q4];

% link lengths 
syms l1 l2 l3 lE
L = [l1 l2 l3 lE]; 

% masses 
% estimate mass values: 
syms m1 m2 m3 m4

% estimate heaviest chess piece: 
syms m_chess

M = [m1 m2 m3 m4 m_chess];

%% Jacobean for end effector: 
J_E = jacobean(Q,L,end_effector_frame); 

%% Jacobean for COM of each link:
J_1 = jacobean(Q,L,1);
J_2 = jacobean(Q,L,2);
J_3 = jacobean(Q,L,3);
J_4 = jacobean(Q,L,4);

%% Symbolic Tau required to hold robot up given particular joint angle: 
Tau = torque_required(Q,L,M);

%% Sub in numbers for symbolic variables - sub in for Q, L and M,m_chess and g
% iteratively solve to get the Tau at each position in workspace 
% figure out what the maximum tau values are for each joint check that they
% are within the correct lengths 

% aribtrary values 
Q = [0.5,0.4,0.3,0.2];
L = [30 35 35 10];
M = [0.4 0.4 0.4 0.5 0.2];
Tau = torque_required(Q,L,M);


