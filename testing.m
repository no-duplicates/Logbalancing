% Define symbolic variables
syms x l0 l1 l2 m0 m1 m2

% Define the equation
eqn = (sin(x) * l1 * m1 + sin(x) * (l1 + l2) * m2) / (m0 + m1 + m2) == l0;

% Solve the equation for x
sol_x = solve(eqn, x);

% Display the solution
disp('The solution for x is:');
disp(sol_x);
