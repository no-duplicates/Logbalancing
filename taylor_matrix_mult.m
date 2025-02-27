function Q = taylor_matrix_mult(A, B)
    % Performs element-wise multiplication of Taylor series given in matrix form.
    % Each column in A and B represents a Taylor series.
    % Returns Q, a matrix where each column is the Taylor series product A(x) * B(x).
    
    [n, m] = size(A); % n: Taylor series order, m: number of Taylor series
    
    if size(B, 1) ~= n || size(B, 2) ~= m
        error('A and B must have the same size.');
    end
    
    Q = sym(zeros(n, m)); % Initialize output matrix
    
    % Compute each Taylor coefficient using the Cauchy product
    for k = 1:n
        for j = 1:m
            Q(k, j) = sum(A(1:k, j) .* B(k:-1:1, j)); % Element-wise multiplication
        end
    end
end
