function [T04] = forward_kinematics_last_joint(Q,PrintStick) 
    %% system parameters 
    % robot angles 
    t1 = Q(1);
    t2 = Q(2); 
    t3 = Q(3); 
    t4 = Q(4);
  
    % link lengths 
    a0  = 0;
    a1 = 0;
    a2 = 30; 
    a3 = 30; 
    d1 = 20; 
    
    %% DH Table 
    DH_Table = [a0 0 d1 t1; a1 pi/2 0 t2; a2 0 0 t3; a3 0 0 t4];

    %% Get the transformation matrices for each row of DH table 

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
    % the transformation matrices T0i can be used for plotting 
    T0i(:,:,1) = Timinus_i(:,:,1); 
    previous = T0i(:,:,1); 

    for i= 2:size(DH_Table) 
        T0i(:,:,i) = previous*Timinus_i(:,:,i);
        previous = T0i(:,:,i) ;
    end 
    
     %% confirmation of forward kinematics      
     if PrintStick == "Print"
        
        n_transforms = 4;
            X = zeros(6);
            Y = zeros(6);
            Z = zeros(6); 
            for i = 1:n_transforms
                X(i+1) = T0i(1,4,i);
                Y(i+1) = T0i(2,4,i);
                Z(i+1) = T0i(3,4,i);
            end 
        % round to prevent any weird rounding errors and plot stick figure 
        plot3(round(X,5),round(Y,5),round(Z,5))  
    end 
    
    %% return the end effector pose 
    T04 = T0i(:,:,4); 

end 