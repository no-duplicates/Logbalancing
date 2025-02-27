function coeff_matrix = taylor_coeff_matrix(F, x, n)
    % Computes the first n Taylor coefficients of an array of symbolic functions.
    %
    % Inputs:
    %   - F: 1×m array of symbolic functions
    %   - x: symbolic variable for expansion
    %   - n: number of Taylor coefficients to extract
    %
    % Output:
    %   - coeff_matrix: n×m matrix where each column corresponds to the coefficients
    %     of the Taylor expansion of the respective function in F.
    
    m = length(F); % Number of functions
    coeff_matrix = sym(zeros(n, m)); % Initialize coefficient matrix
    
    for j = 1:m
        % Compute Taylor expansion for each function
        T = taylor(F(j), x, 'Order', n);
        T=T.';
        % Extract coefficients of the Taylor series
        coeffs_T = fliplr(coeffs(T, x, 'All')); % Ensure correct order
        max_length=min(length(coeffs_T),n);
        % Store coefficients in matrix (truncate/pad as necessary)
        coeff_matrix(1:max_length, j) = [(coeffs_T(1:max_length))];
    end
end
