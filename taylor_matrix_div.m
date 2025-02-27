function Q = taylor_matrix_div(A, B)
    % Performs element-wise multiplication of Taylor series given in matrix form.
    % Each column in A and B represents a Taylor series.
    % Returns Q, a matrix where each column is the Taylor series product A(x) * B(x).
    
    [n, m] = size(A); % n: Taylor series order, m: number of Taylor series
    
    if size(B, 1) ~= n || size(B, 2) ~= m
        error('A and B must have the same size.');
    end
    
    Q = sym(zeros(n, m)); % Initialize output matrix
    for i=1:m
        Q(:,i)=taylor_series_division(A(:,i),B(:,i),n);
    end
    
end
function c_coeffs = taylor_series_division(a_coeffs, b_coeffs,n)
    % Ensure inputs are symbolic

    a_coeffs = sym(a_coeffs);
    b_coeffs = sym(b_coeffs);
    
    % Initialize coefficient vector for c(x)
    c_coeffs = sym(zeros(n,1));
    first_a=find(a_coeffs~=0,1,"first");
    first_b=find(b_coeffs~=0,1,"first");
    if isempty(first_a)
        return;
    end
    if first_b>first_a
        error("b>a, non taylor");
    end
    % Compute first coefficient
    c_coeffs(1-first_b+first_a) = a_coeffs(first_a) / b_coeffs(first_b);
    
    
    % Compute higher order coefficients using the formula
    for k = first_a+1:n
        % Compute q_k using previously computed terms
        c_coeffs(k-first_b+1) = (a_coeffs(k) - dot(c_coeffs(1-first_b+first_a:k-first_b+1-1), b_coeffs(first_b+k-first_a:-1:first_b+1))) / b_coeffs(first_b);
    end
    
end
