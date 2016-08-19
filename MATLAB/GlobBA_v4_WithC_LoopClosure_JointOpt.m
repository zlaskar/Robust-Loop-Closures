%% Function for error calculation 
% function called by main_opt_glob_options_v3
function err_total = GlobBA_v4_WithC_LoopClosure_JointOpt(parm, size_err, R_RGB_Tango, t_RGB_Tango, pix_Cord_Arr, Num3DPts, NViews_3D_Arr, Merge3DProps, switch_constraints, switch_var_size)%Merge3DProps

    %% Init
    err_total = zeros(size_err,1);
    errCounter = 1;

    %% Reshape parm into matrix and components
    switch_merge = parm(end-switch_var_size+1:end);
    
    parm = parm(1:end-switch_var_size);
    param_unpack = reshape( parm, [3,size(parm,1)/3]); % Reshape into 3*M matrix
    param_unpack = param_unpack';

    
    X_3D = param_unpack(1:Num3DPts,:); % Collect the 1st Num3DPts rows which are the world co-ordinates localized
    
    param_pose = param_unpack(Num3DPts+1:end,:);
    
    %% Pose
    t_R = param_pose(1:size(param_pose,1)/2,:);
    C = param_pose(size(param_pose,1)/2+1:end,:);
    
    R=rotationmat(t_R); 
   
    camparams = [1040.47; 1040.63; 634.03700000000003; 365.99900000000002; 0; 0; 0]; %White
    
    % Focal length
    fx = camparams(1);
    fy = camparams(2);
    
    % Principal point
    u0 = camparams(3);
    v0 = camparams(4);
    
    % Distortion parameters
    k1 = camparams(5);
    k2 = camparams(6);
    k3 = camparams(7);
    
    K = [fx  0 u0;
        0 fy v0;
        0  0  1];
    
    errCount = 1;

    %% Compute the respective cost functions
    for i = 1 : size(t_R,1)

        if i > 1 
%             
            %% Rel rotation error

            Rel_rot_MVG = R{i-1}'*R{i};
            Rel_rot_Tango = R_RGB_Tango{i-1}'*R_RGB_Tango{i};
            Rel_Rot_err{1} = Rel_rot_MVG'*Rel_rot_Tango;

            Rel_Rot_vec_err = rotationpars(Rel_Rot_err);
            if norm(Rel_Rot_vec_err) > pi
                Rel_Rot_vec_err = Rel_Rot_vec_err.*(1-(2*pi/norm(Rel_Rot_vec_err)));
            end

            err_total(errCounter:errCounter+2,1) = Rel_Rot_vec_err./(pi/180);
            errCounter = errCounter + 3;
            
            
%             err_RGB_relPose(errCount,1:3) = Rel_Rot_vec_err./(pi/180);
% 
%             errCount = errCount + 1;
            %% Rel translational error
            Rel_center_MVG = R{i-1}'*(C(i,:)' - C(i-1,:)'); 
                        
            Rel_center_Tango = R_RGB_Tango{i-1}'*(t_RGB_Tango{i} - t_RGB_Tango{i-1});
                        
            err_total(errCounter:errCounter+2,1) = (Rel_center_MVG - Rel_center_Tango)./(0.01);
            errCounter = errCounter + 3;
            
%             err_RGB_relPose(errCount,1:3) = (Rel_center_MVG - Rel_center_Tango)./(0.01);
% 
%             errCount = errCount + 1;
        
        else
%             err_RGB_relPose(errCount,1:3) = 0;
%             errCount = errCount + 1;
%             err_RGB_relPose(errCount,1:3) = 0;
%             errCount = errCount + 1;
            err_total(errCounter:errCounter+5,1) = 0;
            errCounter = errCounter + 6;
        
        end  
    end
% % %         %% SIFT reprojection error

% %     %% Loop over the views/ Images seen by the 3D point (better than checking if each image is viewing the 3D point in the above loop)
    errCountRGB = 1;
    switch_Count = 1;
    for sz = 1 : size(NViews_3D_Arr,2)
        NViews_3D = NViews_3D_Arr{sz};

        pix = pix_Cord_Arr{sz};
        for k = 1 : numel(NViews_3D)

            % Loop over the 3D points seen by current camera
            %             disp (k)
            i = NViews_3D(k); % The k th image viewing the 3D point X_3D
%             j = NView_3D_Index(k);
            pix_cur = pix(k,:);
            
            pixCam_est = K*(R{i}'*(X_3D(sz,:))'+(-R{i}'*(C(i,:)'))); % 3*1 vec
            est_dep = pixCam_est(3);
            pixCam_est = pixCam_est./pixCam_est(3);
            %                 pixCam_est = K*(R{i}'*(X_3D)'+(-R{i}'*(C(i,:)'./ThreeD_ImgStruct_Global.view{i}.scale))); % 3*1 vec
            pixCam_obv = pix_cur./(pix_cur(3));
            obv_dep = pixCam_obv(3);
            
            err_proj_dep(errCountRGB) = est_dep - obv_dep;
%             err_RGB(errCountRGB,1:2) = switch_constraints(errCountRGB)*((pixCam_obv(1:2) - pixCam_est(1:2)'));
            err_total(errCounter:errCounter+1,1) = switch_merge(switch_Count)*((pixCam_obv(1:2) - pixCam_est(1:2)'))/.1;
            errCounter = errCounter + 2;
% Old equation
%            err_RGB(errCountRGB,1:2) = switch_merge(errCountRGB)*((pixCam_obv(1:2) - pixCam_est(1:2)'));
            %err_RGB(end+1,1:2) = ((pixCam_obv(1:2)' - pixCam_est(1:2)));
            switch_Count = switch_Count + 1;
            errCountRGB = errCountRGB + 1;
            % % %             err_RGB(i,1) = err + err_RGB(i);
            
            %             end
            %         end
        end
    end

    %% Coompute error in closing the loop between the threePtNum & threePtNum - relocInd th 3D point 
    errCountMerge = 1;
    init = 1;
%     w = [0 0 0.1];
    for CopyParents = 1 : numel(Merge3DProps)
    Copies = Merge3DProps{CopyParents};
    NumCopies = (Copies);
    for i = init : init + NumCopies - 2
        for j = i + 1 : init + NumCopies - 1
            J_col_Count_base_1 = (i);
%             J_col_Count_base_2 = (init-1 + i-j);
            J_col_Count_base_2 = (j);
            
            err_total(errCounter:errCounter+2,1) = ((switch_merge(switch_Count)*1*(X_3D(J_col_Count_base_1,:) - X_3D(J_col_Count_base_2,:))));
            errCounter = errCounter + 3;
            % Olde eq
            %err_euclid_LoopClos(errCountMerge,:) = switch_merge(switch_Count)*1*(X_3D(J_col_Count_base_1,:) - X_3D(J_col_Count_base_2,:));
%             w(errCountMerge)*
            errCountMerge = errCountMerge + 1;
            switch_Count = switch_Count + 1;
        end
    end
    init = NumCopies + init;
    end
    
    %% Coompute error in priors penalty
    errCnt = 0;
    for kk = 1 : switch_var_size
       errCnt = errCnt + 1;
       err_total(errCounter,1) = 1 - switch_merge(errCnt);
%        err_total(errCounter,1) = 0.01*(log(switch_merge(errCnt)) - switch_merge(errCnt)) ;
       errCounter = errCounter + 1;
       %old eq 
       %errPenPriors(errCnt, 1) = 1 - switch_merge(errCnt);
        
    end
    
    
% %     err_pose = reshape(err_RGB_relPose',[numel(err_RGB_relPose),1]);
% %     err2 = reshape(err_RGB',[numel(err_RGB),1]);
% % % % %         err2 = reshape(err_RGB',[1,size(err_RGB,1)*size(err_RGB,2)]);
% %     err2 = err2./0.1;
% %     
% %     if errCountMerge > 2 % If loop closure is present
% %         
% %         err_LC = reshape(err_euclid_LoopClos',[numel(err_euclid_LoopClos),1]);
% %         err_total = [double(err_pose); double(err2); double(err_LC); double(errPenPriors)];%double(err_LC);
% %     else
% %         err_total = [double(err_pose); double(err2);double(errPenPriors)];
% %     end

% % %     sum(err_total)
% % %     clear var R_RGB_Tango t_RGB_Tango ThreeD_ImgStruct_Global
    

end