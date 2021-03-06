%% Written by M.Trollip, B.De Worsop and N.Parry

    % 'forward_kinematics.m' takes 4 arguments: 
    % - 1st: the joint space Q, 
    % - 2nd: a string to specify if you want to print a stick figure, enter
    %   'Print' to output a stick figure 
    % - 3rd: the frame number you wish to output (i.e for end effector, would
    % - enter 5) 
    % - 4th: the link lengths  
    % The function returns the pose for a specified frame. It can also
    % print a stick figure if specified. 

function [T0F] = forward_kinematics(Q,PrintStick,L,frame, tau)
    %% system parameters 
    % robot angles 
    t1 = Q(1);
    t2 = Q(2); 
    t3 = Q(3);
    t4 = Q(4);
    
    % sub in parameters 
    a0  = 0;
    a1 = 0;
    a2 = L(2); 
    a3 = L(3); 
    d1 = L(1);
    dE = L(4);

    
    %% DH Table 
    DH_Table = [a0 0 d1 t1; a1 pi/2 0 t2; a2 0 0 t3; a3 0 0 t4; ...
        0 pi/2 dE 0];

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

    %% Get the transformation matrices to express the joints from origin 
    % the transformation matrices T0i can be used for plotting 
    T0i(:,:,1) = Timinus_i(:,:,1); 
    previous = T0i(:,:,1); 

    for i= 2:size(DH_Table) 
        T0i(:,:,i) = previous*Timinus_i(:,:,i);
        previous = T0i(:,:,i) ;
    end 
    
    
     %% Print Stick Figure if specified    
     if PrintStick == "Print"
         
         if T0i(3,4,5) < 0
            %do nothing 
         else
        
        n_transforms = frame; 
            X = [0];
            Y = [0];
            Z = [0]; 
            for i = 1:n_transforms
                X(i+1) = T0i(1,4,i);
                Y(i+1) = T0i(2,4,i);
                Z(i+1) = T0i(3,4,i);
            end 
%         figure 
%         set(gca,'FontSize', 14)
%         set(gcf,'color','w');
%         

        % round to prevent any weird rounding errors and plot stick figure 
        plot3(round(X,5),round(Y,5),round(Z,5), 'color',[.5 .4 .7]) % the link length 
        xlabel('x_0 Displacement (cm)','FontSize',14)
        ylabel('y_0 Displacement (cm)','FontSize',14)
        zlabel('z_0 Displacement (cm)','FontSize',14)
        title('Plot torque values based on robot configuration','FontSize',40) 


        grid on
        hold on
        
%         real_cols = [255,((max(abs(tau)))/1.6) * 255,((max(abs(tau)))/1.6) * 255]
%         RGB = validatecolor(uint8([255,255- (((max(abs(tau)))/1.6) * 255), 255 - (((max(abs(tau)))/1.6) * 255)]));
        RGB = validatecolor(uint8([255 - (((max(abs(tau)))/1.6) * 255),255- (((max(abs(tau)))/1.6) * 255), 255 - (((max(abs(tau)))/1.6) * 255)]));
        plot3(round(X,5),round(Y,5),round(Z,5),'o','MarkerFaceColor', RGB); % the blue joint circle
       
%         left_labels = {'Joint 1/Base','Joint 2','Joint 2', 'Joint 3'}; 
%         right_labels = {'Joint 4', 'End Effector'};
        % joints with labels on 
        % some on the left and some on the right of the joints depending on
        % figure 
%         text(round(X(1:4),5),round(Y(1:4),5),round(Z(1:4),5),left_labels,'VerticalAlignment','top','HorizontalAlignment','left','FontSize',14)
%         text(round(X(5:6),5),round(Y(5:6),5),round(Z(5:6),5),right_labels,'VerticalAlignment','top','HorizontalAlignment','right','FontSize',14)
%         hold off 
        
        % these here - causing issues with print stick
        xlim([-0.1 0.1])
        ylim([-0.1 0.1])
        view([0,0])     

        end 
    end
    
    %% return the pose of a particular joint 
    
    % note if frame = 5 then return end effecter i.e solve forward
    % kinematics
    
    T0F = T0i(:,:,frame);
     
end
   
