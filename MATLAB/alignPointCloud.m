function [ XYZ_A ] = alignPointCloud( XYZ )
%ALIGNPOINTCLOUD Rotates and centers the points based on variance
%

%% Step 1. Center the data at zero:
xyz0=mean(XYZ);
A=bsxfun(@minus,XYZ,xyz0); %center the data
%scatter3(A(:,1),A(:,2),A(:,3));

%% Step 2. Find the direction of most variance using SVD and rotate the data to make that the x axis.
[~,~,V]=svd(A,0);
A_rot = A*V; %V(:,1) is the direction of most variance
%hold on, scatter3(A_rot(:,1),A_rot(:,2),A_rot(:,3));

%% Step 3. Slide the data up the x axis so all the points are x >= 0.
XYZ_A = bsxfun(@minus,A_rot,[min(A_rot(:,1)) 0 0]);
%scatter3(XYZ_A(:,1),XYZ_A(:,2),XYZ_A(:,3));

end
