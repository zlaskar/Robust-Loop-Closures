%% Function to construct Jacobian Pattern
% Author : Zakaria Laskar
% Date : 12_09_2015

function [JacoPat, size_err] = construct_Jacob_Pattern_LoopClosure_JointOpt( Num_Poses, NViews_Arr, exclude, size_param, threePtNum,Merge3DProps, pairsOf3D, totalEdges, switch_var_size)%Merge3DProps

%     Num_NViews = numel(NViews_Arr);
%     JacoPat = zeros( (Num_Poses - numel(exclude))*6 + Num_NViews*2, size_param);
%     JacoPat = zeros( (Num_Poses)*6 + Num_NViews*2, size_param);
%     ones((size(temp_Rot_RGB,1)-1)*6+ numel(NViews_3D)*2,3+numel(t_vec)*2);
%     switch_var_size = pairsOf3D + totalEdges;
    
    err_terms_rel_pose = Num_Poses*6;
    err_terms_reproj = totalEdges*2;
    err_terms_merge_3D = pairsOf3D*3;
    err_terms_switch_penalty = switch_var_size*1;
    size_err = err_terms_rel_pose+ err_terms_reproj + err_terms_merge_3D + err_terms_switch_penalty;
    JacoPat = zeros( size_err, size_param);
    
    
    errCount = 0;
    Center_pos = Num_Poses * 3;
    J_row_Count = 0;
    ptsPlusPoses = threePtNum*3 + Center_pos*2;
    %% Jacobian pattern for rel pose error
    for iter = 1 : Num_Poses
        if iter == exclude(1)
%             || iter == exclude(2)
            J_row_Count = J_row_Count + 6;
            continue;
        end
%         J_row_Count_base = errCount * 6;
        J_col_Count_base = (iter - 1 + threePtNum ) * 3; 
        for i = 1 : 6 % 3-R + 3-C
            J_row_Count = J_row_Count + 1;
%             J_row_Count = J_row_Count_base + i
            if i < 4
                JacoPat(J_row_Count, J_col_Count_base - 2:J_col_Count_base + 3 ) = 1;
            else
                JacoPat(J_row_Count, J_col_Count_base - 2:J_col_Count_base ) = 1;
                JacoPat(J_row_Count, J_col_Count_base + Center_pos - 2:J_col_Count_base + Center_pos + 3 ) = 1;
            end
        end
        errCount = errCount + 1;
    end
    
    %% Jacobian pattern for rerpojection error
%     for k = 1 : size(NV,2)
%         NViews_Arr = NV{k};
%         viewCount = 0;
%         pix_3DJacCount = 1;
%         for sz = 1 : size(NViews_Arr,2)
%             NViews = NViews_Arr{sz};
%             %     for sz = 1 : size(NViews_Arr,1)
%             %         NViews = NViews_Arr(sz,:);
%             for iter = 1 : numel(NViews)
% %                 -1:numel(NViews)
%                 viewCount = viewCount + 1;
%                 Pose_Num = NViews(iter);
%                 %         J_row_Count_base = errCount * 6;
%                 J_col_Count_base = (Pose_Num + threePtNum - 1) * 3;
%                 for i = 1 : 2 % 2-PixelCords (x,y)
%                     
%                     J_row_Count = J_row_Count + 1;
%                     JacoPat(J_row_Count, J_col_Count_base + 1:J_col_Count_base + 3 ) = 1;
%                     JacoPat(J_row_Count, J_col_Count_base + Center_pos + 1:J_col_Count_base + Center_pos + 3 ) = 1;
% %                     JacoPat(J_row_Count, 1:3 ) = 1;
%                     JacoPat(J_row_Count, pix_3DJacCount:pix_3DJacCount+2 ) = 1;
%                     
%                 end
%                 
%             end
%             pix_3DJacCount = pix_3DJacCount + 3;
%         end
%     end
    
%     %% Jacobian for the euclidian distance between 3D point threePtNum & threePtNum - relocInd
%     init = 1;
%     for CopyParents = 1 : numel(Merge3DProps)
%     Copies = Merge3DProps{CopyParents};
%     NumCopies = (Copies);
%     for i = init + 1 : init + NumCopies - 1
%         for j = init : i - 1
%             J_col_Count_base_1 = (i-1)*3;
%             J_col_Count_base_2 = (init-1 + i-j-1)*3;
%             for k = 1 : 3
%                 J_row_Count = J_row_Count + 1;
%                 JacoPat(J_row_Count, J_col_Count_base_1 + 1: J_col_Count_base_1 + 3) = 1;
%                 JacoPat(J_row_Count, J_col_Count_base_2 + 1: J_col_Count_base_2 + 3) = 1;
%             end
% %     JacoPat = JacoPat';
%         end
%     end
%     init = NumCopies + init;
%     end
    
       viewCount = 0;
        pix_3DJacCount = 1;
        for sz = 1 : size(NViews_Arr,2)
            NViews = NViews_Arr{sz};
            %     for sz = 1 : size(NViews_Arr,1)
            %         NViews = NViews_Arr(sz,:);
            for iter = 1 : numel(NViews)
%                 -1:numel(NViews)
                viewCount = viewCount + 1;
                Pose_Num = NViews(iter);
                %         J_row_Count_base = errCount * 6;
                J_col_Count_base = (Pose_Num + threePtNum - 1) * 3;
                for i = 1 : 2 % 2-PixelCords (x,y)
                    
                    J_row_Count = J_row_Count + 1;
                    JacoPat(J_row_Count, J_col_Count_base + 1:J_col_Count_base + 3 ) = 1;
                    JacoPat(J_row_Count, J_col_Count_base + Center_pos + 1:J_col_Count_base + Center_pos + 3 ) = 1;
                    JacoPat(J_row_Count, ptsPlusPoses + viewCount) = 1;
%                     JacoPat(J_row_Count, 1:3 ) = 1;
                    JacoPat(J_row_Count, pix_3DJacCount:pix_3DJacCount+2 ) = 1;
                    
                end
                
            end
            pix_3DJacCount = pix_3DJacCount + 3;
        end
%     end
    
    % Jacobian for the euclidian distance between 3D point threePtNum & threePtNum - relocInd
    init = 1;
%     viewCount = 0;
    for CopyParents = 1 : numel(Merge3DProps)
    Copies = Merge3DProps{CopyParents};
    NumCopies = (Copies);
    for i = init : init + NumCopies - 2
        for j = i + 1: init + NumCopies - 1
            J_col_Count_base_1 = (i-1)*3;
%             J_col_Count_base_2 = (init-1 + i-j-1)*3;
            J_col_Count_base_2 = (j-1)*3;
            viewCount = viewCount + 1;
            for k = 1 : 3
                J_row_Count = J_row_Count + 1;
                JacoPat(J_row_Count, J_col_Count_base_1 + 1: J_col_Count_base_1 + 3) = 1;
                JacoPat(J_row_Count, J_col_Count_base_2 + 1: J_col_Count_base_2 + 3) = 1;
                JacoPat(J_row_Count, ptsPlusPoses + viewCount) = 1;
            end
%     JacoPat = JacoPat';
        end
    end
    init = NumCopies + init;
    end
   
    %% penalty priors
    viewCount = 0;
    for kk = 1 : switch_var_size
       J_row_Count = J_row_Count + 1;
       viewCount = viewCount + 1;
       JacoPat(J_row_Count, ptsPlusPoses + viewCount) = 1;
        
    end
    
%     for iter = 1 : Num_Poses
%         if iter == exclude(1)
% %             || iter == exclude(2)
%             J_row_Count = J_row_Count + 6;
%             continue;
%         end
% %         J_row_Count_base = errCount * 6;
%         J_col_Count_base = (iter - 1 + threePtNum ) * 3; 
%         for i = 1 : 6 % 3-R + 3-C
%             J_row_Count = J_row_Count + 1;
% %             J_row_Count = J_row_Count_base + i
%             if i < 4
%                 JacoPat(J_row_Count, J_col_Count_base - 2:J_col_Count_base + 3 ) = 1;
%             else
%                 JacoPat(J_row_Count, J_col_Count_base - 2:J_col_Count_base ) = 1;
%                 JacoPat(J_row_Count, J_col_Count_base + Center_pos - 2:J_col_Count_base + Center_pos + 3 ) = 1;
%             end
%         end
%         errCount = errCount + 1;
%     end
%     
%     pix_3DJacCount = 1;
%         for sz = 1 : size(NViews_Arr,2)
%             NViews = NViews_Arr{sz};
%             %     for sz = 1 : size(NViews_Arr,1)
%             %         NViews = NViews_Arr(sz,:);
%             for iter = 1 : numel(NViews)
% %                 -1:numel(NViews)
%                 viewCount = viewCount + 1;
%                 Pose_Num = NViews(iter);
%                 %         J_row_Count_base = errCount * 6;
%                 J_col_Count_base = (Pose_Num + threePtNum - 1) * 3;
%                 for i = 1 : 2 % 2-PixelCords (x,y)
%                     
%                     J_row_Count = J_row_Count + 1;
%                     JacoPat(J_row_Count, J_col_Count_base + 1:J_col_Count_base + 3 ) = 1;
%                     JacoPat(J_row_Count, J_col_Count_base + Center_pos + 1:J_col_Count_base + Center_pos + 3 ) = 1;
% %                     JacoPat(J_row_Count, 1:3 ) = 1;
%                     JacoPat(J_row_Count, pix_3DJacCount:pix_3DJacCount+2 ) = 1;
%                     
%                 end
%                 
%             end
%             pix_3DJacCount = pix_3DJacCount + 3;
%         end
%     
    
    
    
    
    
    
    
    
    
    
end









