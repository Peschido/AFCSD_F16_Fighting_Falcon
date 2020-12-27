function f = latex_creator(A, which)
    latex_code = '\begin{bmatrix} ';
    if which == 1;
        A(7,:) = [];
        A(:,7) = [];
        A(6,:) = [];
        A(:,6) = [];
        A(1,:) = [];
        A(:,1) = []; 
        
        A([1 3],:)= A([3 1],:);
        A([2 1],:)= A([1 2],:);
        A(:,[1 3]) = A(:,[3 1]);
        A(:,[2 1]) = A(:,[1 2]);
        
        j = 0;
        while j < size(A,1);
            j = j + 1;
            ii = 0;
            for i = A(j,:)
                ii = ii + 1;
                if ii+1 <= size(A,1);
                    latex_code = append(latex_code, string(round(i,4)),' & ');           
                else 
                    latex_code = append(latex_code, string(round(i,4)), ' \\ ');
                end
            end
        end
                
    end 
    if which == 2;
        A(1,:) = [];
        A(5,:) = [];
        A(:,1) = [];  
        ii = 1;
        while ii < 6;
            j = A(ii,:);
            if ii <= 4;
               latex_code = append(latex_code, string(round(j,4)),' \\ ');           
            else 
               latex_code = append(latex_code, string(round(j,4)), ' ');
            end
            ii = ii + 1;
        end
    end
    
    if which == 3;
        A(1,:) = [];
        A(:,1) = [];
        A([1 3],:)= A([3 1],:);
        A([2 1],:)= A([1 2],:);
        A(:,[1 3]) = A(:,[3 1]);
        A(:,[2 1]) = A(:,[1 2]);
       
        j = 0;
        while j < size(A,1);
            j = j + 1;
            ii = 0;
            for i = A(j,:)
                ii = ii + 1;
                if ii -1 <= size(A,1);
                    latex_code = append(latex_code, string(round(i,4)),' & ');           
                else 
                    latex_code = append(latex_code, string(round(i,4)), ' \\ ');
                end
            end
        end
                
    end 
    if which == 4;
        %A(1,:) = [];
        %A(5,:) = [];
        A(:,1) = [];  
        ii = 1;
        while ii < 6;
            j = A(ii,:);
            if ii <= 4;
               latex_code = append(latex_code, string(round(j,4)),' \\ ');           
            else 
               latex_code = append(latex_code, string(round(j,4)), ' ');
            end
            ii = ii + 1;
        end
    end
    
    if which == 5;
        A(1,:) = [];
        A(5,:) = [];
        A(:,1) = [];  
        ii = 1;
        
    end
    if which == 6;
        A(1,:) = [];
        A(5,:) = [];
        A(:,1) = [];  
        ii = 1;
        
    end
    if which == 7;
        A(1,:) = [];
        A(5,:) = [];
        A(:,1) = [];  
        ii = 1;
        
    end
    if which == 8;
        A(1,:) = [];
        A(5,:) = [];
        A(:,1) = [];  
        ii = 1;
        
    end
    
    
    latex_code = append(latex_code, ' \end{bmatrix}');
    f = A;
end



