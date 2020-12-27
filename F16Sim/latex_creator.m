function f = latex_creator(A)
    B = matrix_reducer(A, [1, 5]);
    C = matrix_swapper(B);
    latex_code = '\begin{bmatrix} ';
    j = 0;
    while j < size(C,1);
        j = j + 1;
        ii = 0;
        for i = C(j,:)
            ii = ii + 1;
            if ii+1 <= size(C,1);
                latex_code = append(latex_code, string(round(i,4)),' & ');           
            else 
                latex_code = append(latex_code, string(round(i,4)), ' \\ ');
            end
        end
    end
    latex_code = append(latex_code, ' \end{bmatrix}');
    f = latex_code; 
    
end

function ff = matrix_reducer(A, deletions)
    for i = deletions;
        A(i,:) = [];
        A(:,i) = [];
    end 
    ff = A;
end

function fff = matrix_swapper(A)
    A([1 3],:)= A([3 1],:);
    A([2 1],:)= A([1 2],:);
    A(:,[1 3]) = A(:,[3 1]);
    A(:,[2 1]) = A(:,[1 2]);
    fff = A;
end 