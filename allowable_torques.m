%% iteratively go through link length dimensions to see when the torque is less than 1.5 Nm for motor 3 

L = [0.3 0.35 0.36 0.1];

% masses and angle remains constant 
M = [0.1 0.11 0.12 0.01 0.086 0.086 0.05];
Q = [0;0;0;0]; 

tau  = torque_required(Q,L,M);

allowable_tau = 1.5;
while abs(tau(2)) > allowable_tau 
    L(2) = L(2) - 0.01;
    L(3) = L(3) -0.01; 
    tau = torque_required(Q,L,M);
end 

% maximum link lenghts are: 
disp(L)
disp(double(tau)) 