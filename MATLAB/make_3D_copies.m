%% Function to cluster the images seen by the 3D point
% Date 1/10/2015
% Author : Zakaria Laskar

function [ Local_Copies] = make_3D_copies( pix_cord, NViews, partition)

    %% Iterate over the Views and do online clustering
    numViews = numel(NViews);
    copyCnt = 0;

    for i = 1 : numViews
        % if diff between curr n prev frame is less than 30 add to cluster
       if  i > 1 && NViews(i) - NViews(i-1) < partition % i > 1 && i < numViews &&
        
           viewCnt = viewCnt + 1;
           Local_Copies.copy{copyCnt}.pixCord(viewCnt,:) = pix_cord(i,:);
%            Local_Copies.copy{copyCnt}.NViews_Index(viewCnt) = NViews_Index(i);
           Local_Copies.copy{copyCnt}.NViews(viewCnt) = NViews(i);
           
        % if a new cluster do init
       else
            
           copyCnt = copyCnt + 1;
           viewCnt = 1;
           Local_Copies.copy{copyCnt}.pixCord(viewCnt,:) = pix_cord(i,:);
%            Local_Copies.copy{copyCnt}.NViews_Index(viewCnt) = NViews_Index(i);
           Local_Copies.copy{copyCnt}.NViews(viewCnt) = NViews(i);
       end
        
    end















end