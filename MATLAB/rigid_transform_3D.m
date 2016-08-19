% This function finds the optimal Rigid/Euclidean transform in 3D space
% It expects as input a Nx3 matrix of 3D points.
% It returns R, t

% You can verify the correctness of the function by copying and pasting these commands:
%{

R = orth(rand(3,3)); % random rotation matrix

if det(R) < 0
    V(:,3) *= -1;
    R = V*U';
end

t = rand(3,1); % random translation

n = 10; % number of points
A = rand(n,3);
B = R*A' + repmat(t, 1, n);
B = B';

[ret_R, ret_t] = rigid_transform_3D(A, B);

A2 = (ret_R*A') + repmat(ret_t, 1, n)
A2 = A2'

% Find the error
err = A2 - B;
err = err .* err;
err = sum(err(:));
rmse = sqrt(err/n);

disp(sprintf("RMSE: %f", rmse));
disp("If RMSE is near zero, the function is correct!");

%}

% expects row data
function [R,t] = rigid_transform_3D(A, B)
    if nargin != 2
	    error("Missing parameters");
    end

    assert(size(A) == size(B))

    centroid_A = mean(A);
    centroid_B = mean(B);

    N = size(A,1);

    H = (A - repmat(centroid_A, N, 1))' * (B - repmat(centroid_B, N, 1));

    [U,S,V] = svd(H);

    R = V*U';

    if det(R) < 0
        printf("Reflection detected\n");
        V(:,3) *= -1;
        R = V*U';
    end

    t = -R*centroid_A' + centroid_B'
end

