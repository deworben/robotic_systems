%% Symbolic variables 
% joint angles 
syms q1 q2 q3 q4
Q = [q1 q2 q3 q4];

% link lengths 
syms l1 l2 l3 lE
L = [l1 l2 l3 lE]; 

% masses 
% estimate link mass values: 
syms m_l1 m_l2 m_l3 m_l4

% estimate motor 3 and 4 mass values:
syms m_mot3 m_mot4

% estimate heaviest chess piece: 
syms m_chess

M = [m_l1 m_l2 m_l3 m_l4 m_mot3 m_mot4 m_chess];

%% Jacobian for end effector:
end_effector_frame = 5;

J_E = jacobean_joint(Q,L,end_effector_frame); 

%% Jacobian for COM of each link:
J_1 = jacobean_Link(Q,L,1);
latex(J_1(1:3,:));

J_2 = jacobean_Link(Q,L,2);
latex(J_2(1:3,:));

J_3 = jacobean_Link(Q,L,3);
J_3 = simplify(J_3);
latex(J_3(1:3,:));

J_4 = jacobean_Link(Q,L,4);
J_4 = simplify(J_4);
latex(J_4(1:3,:));

%% Jacobian for each motor
J_motor1 = jacobean_joint(Q,L,1);
J_motor1 = simplify(J_motor1);
latex(J_motor1(1:3,:));

J_motor2 = jacobean_joint(Q,L,2);
J_motor2 = simplify(J_motor2);
latex(J_motor2(1:3,:));

J_motor3 = jacobean_joint(Q,L,3);
J_motor3 = simplify(J_motor3);
latex(J_motor3(1:3,:));

J_motor4 = jacobean_joint(Q,L,4);
J_motor4 = simplify(J_motor4);
latex(J_motor4(1:3,:));

%% Symbolic Tau required to hold robot up given particular joint angle: 
% Tau = torque_required(Q,L,M);

%% Sub in numbers for symbolic variables - sub in for Q, L and M,m_chess and g
% do this in 'plot_torques.m'. So that have no symbolic variables 

Q = [0,0,0,0];
L = [0.3 0.35 0.36 0.1];
M = [0.1 0.11 0.12 0.01 0.086 0.086 0.07];

Tau = vpa(torque_required(Q,L,M))


% Q = [0,0,0,0.5]
% vpa(torque_required(Q,L,M))

