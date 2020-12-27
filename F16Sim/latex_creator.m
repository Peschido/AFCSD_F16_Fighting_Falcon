function f = latex_creator(A, which)
    if which == 1
        for i = [1, 5];
            A(i,:) = [];
            A(:,i) = [];
        end 
        A([1 3],:)= A([3 1],:);
        A([2 1],:)= A([1 2],:);
        A(:,[1 3]) = A(:,[3 1]);
        A(:,[2 1]) = A(:,[1 2]);
       
        latex_code = '\begin{bmatrix} ';
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
        latex_code = append(latex_code, ' \end{bmatrix}');
        f = latex_code; 
    end 
    
end



