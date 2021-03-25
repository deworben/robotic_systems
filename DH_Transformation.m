function [T_01] = DH_Transformation(a0,alpha0, d1, t1) 

Dx = [1 0 0 a0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

Rx = [1 0 0 0; 0 cos(alpha0) -sin(alpha0) 0; 0 sin(alpha0) cos(alpha0) 0; 0 0 0 1]; 
    
Dz = [1 0 0 0; 0 1 0 0; 0 0 1 d1; 0 0 0 1];

Rz = [cos(t1) -sin(t1) 0 0; sin(t1) cos(t1) 0 0; 0 0 1 0; 0 0 0 1];

T_01 = Dx*Rx*Dz*Rz;

end 