function [C] = symbolic_cross(A,B) 

    C_1 = A(2)*B(3) - A(3)*B(2);
    C_2 = A(3)*B(1)- A(1)*B(3);
    C_3 = A(1)*B(2) - A(2)*B(1);
    
    C = [C_1;C_2;C_3];
end 