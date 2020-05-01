function [yy, total_error] = FitAndError(data_walk)

    % fitting
    p = polyfit(data_walk(:, 1), data_walk(:, 2), 1);
    yy = polyval(p,data_walk(:, 1));

    % error
    A = p(1);
    B = -1;
    C = p(2);
    total_error = [];
    for i = 1 : size(data_walk, 1)
        x = data_walk(i, 1);
        y = data_walk(i, 2);
        error = (A*x + B*y + C) / sqrt(A^2 + B^2);
        total_error = [total_error, error];
    end
    
end