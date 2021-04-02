%% Forward Kinematics 
fprintf('--------------- Forward Kinematics -------------------\n\n') 
% symbolic variables, q are joint angles, l are link lengths  
syms t q1(t) q2(t) q3(t) q4(t) l1 l2 l3 lE 

End_Effector_Frame = 5;

% Task 1: produce transformation matrices of frame i expressed in i-1
%         and show final transformation matrix w.r.t inertial frame 

    % First: create the DH Table 
     
     % symbolically assign the variables 
     a0 = 0;
     a1 = 0;
     a2 = l2; 
     a3 = l3; 
     d1 = l1;
     dE = lE;
     
     t1 = q1;
     t2 = q2;
     t3 = q3;
     t4 = q4;
     
     DH_Table = [a0 0 d1 t1; a1 pi/2 0 t2; a2 0 0 t3; a3 0 0 t4;...
         0 pi/2 dE 0];

    % Second: iteratively call DH_Transformation to get the necessary 
    %         transformation matrices, stored in an array of 4x4 matrices 
    
    for i = 1 : size(DH_Table,1)
        % extract paramaters from DH_Table 
        a_iminus = DH_Table(i,1);
        alph_iminus = DH_Table(i,2);
        d_i = DH_Table(i,3);
        t_i = DH_Table(i,4); 
        % transformation i expressed in i-1
        Timinus_i(:,:,i) = DH_Transformation(a_iminus, alph_iminus, ...
            d_i, t_i); 
    end 
    
    % Third: multiply through the previously calculated transformation
    %        matrices to get the end effector position in frame {0} 
    
    % get the pose for each frame expressed in frame{0}. 
    % Store them in an array of 4x4 matrices
    
    % frame {1} expressed in frame {0} is 
    T0i(:,:,1) = Timinus_i(:,:,1); 
    % store this for the loop 
    previous = T0i(:,:,1); 
    
    % loop through to get the pose for each frame expressed in the inertial 
    % frame. The last frame will be the end effector
     
    for i = 2:End_Effector_Frame
        % frame {i} expressed in frame {0} 
        T0i(:,:,i) = previous*Timinus_i(:,:,i);
        previous = T0i(:,:,i);
    end 
    
    %The end effector in frame {0} is therefore 
    fprintf('The symbolic solution to the robots forward kinematics is:\n');
    T0E = simplify(T0i(:,:,End_Effector_Frame));
    disp(T0E);
    
    diff(T0E,t)