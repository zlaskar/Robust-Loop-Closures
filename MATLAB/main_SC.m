function [Switch_merge, tx_sub, tx_sub_AL, Opt_Rot_RGB, Opt_C_RGB, t_subsampled_RGB, C_RGB, t_subsampled_RGB_AL, C_RGB_AL, RGB4Pts_subsampled, RGB4Pts_AL_subsampled] = main_SC(ThreeD_ImgStruct_Global, ThreeD_Pts_Global, R_RGB_Tango, t_RGB_Tango, SigPtWorldInd, TangoTxPoints, R_RGB_Tango_AL, t_RGB_Tango_AL, TangoTxPoints_AL, RGB4Pts, RGB4Pts_AL,sampleRate, NumOfClus, partition, minNumViewsPer3Dpt)

% % % % % % main script for global optimization


% load Esa_1625_main.mat
% load ZRoom_main.mat
% load ZRoom_main_sampled
% load CMV_main_sampled
% load Esa_981_main.mat
% load Esa_981_Final
% load Wrong_loop_closure_local_Final
% load Esa_2019_Final
% load Oulu_Univ_sampled_Final_2
% load('2253_3D_stdDev_main')
% load Wrong_loop_closure_local_main.mat
% load Esa_2019_main_SS_v2
% load Wrong_loop_closure_v3
% load Wrong_loop_closure_main_v2
% load ('initGlobOpt_Incremental_3D_unscaled.mat');
% load('initGlobOpt_Incremental_3D_14_09_2015_1');
% load('initGlobOpt_Incremental_3D_14_09_2015_Juho_init');
% tmp = ThreeD_Pts_Global;
% tmp1 = ThreeD_ImgStruct_Global; 
% load('initGlobOpt_unscaled')
% ThreeD_Pts_Global = tmp;
% ThreeD_ImgStruct_Global = tmp1;
% load ('TxPtsGlobal');
%% 1st encode the 3D points
NumImages = numel(R_RGB_Tango);
subsampleSet = [1:sampleRate:NumImages];
t_vec_RGB_Tango = double(rotationpars(R_RGB_Tango));
t_vec_RGB_Tango_AL = double(rotationpars(R_RGB_Tango_AL));
limit = 6*numel(subsampleSet);
%% camera intrinsics and distortion params
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


%% Create an indexing 

% index_3D_subset = zeros(1,size(ThreeD_Pts_Global.pts,2));
threeD_subset_counter = 1;

paramCount = 1;
interval = 1;

for i =  1 :interval:size(subsampleSet,2)   

%    C(paramCount,:) = double(t_MVG_Tango{1,subsampleSet(i)});
   C_RGB(paramCount,:) = double(t_RGB_Tango{1,subsampleSet(i)});
   C_RGB_AL(paramCount,:) = double(t_RGB_Tango_AL{1,subsampleSet(i)});
%    t_subsampled(paramCount,:) = t_vec_MVG_Tango(subsampleSet(i),:);
   t_subsampled_RGB(paramCount,:) = t_vec_RGB_Tango(subsampleSet(i),:);
   t_subsampled_RGB_AL(paramCount,:) = t_vec_RGB_Tango_AL(subsampleSet(i),:);
   tx_sub{paramCount} = TangoTxPoints{1,subsampleSet(i)};
   tx_sub_AL{paramCount} = TangoTxPoints_AL{1,subsampleSet(i)};
   RGB4Pts_subsampled{paramCount} = RGB4Pts{1,subsampleSet(i)};
   RGB4Pts_AL_subsampled{paramCount} = RGB4Pts_AL{1,subsampleSet(i)};
   org_cen(paramCount,:) = double(t_RGB_Tango{1,subsampleSet(i)});
   cen_Tango{paramCount} = (t_RGB_Tango{1,subsampleSet(i)});
   Rot_Tango{paramCount} = (R_RGB_Tango{1,subsampleSet(i)});
   
   % tmp
%    RGB4Pts_subsampled{paramCount} = TangoRGB4PtCld{1,subsampleSet(i)};
   paramCount = paramCount + 1;
   
end



%% Algo Start 

total3D_Pts = 1;
Opt_Rot_RGB = t_subsampled_RGB;
Opt_C_RGB = C_RGB;
% NumOfClus = 7;
% partition = 15;
% minNumViewsPer3Dpt = 2;
%% Iter over the 3D points
Iter = 1;
threeDArray = [3];
% threeDArray = [3];
threeD_Cnt_instance = 0;
viewIncr = 0;
mergeIncr = 0;
mergeCount = 0;
total3DptsPerClus_tmp = 0;
for clus = 1 : NumOfClus
    if numel(SigPtWorldInd{(clus)}) < 30
        total3DptsPerClus_tmp = numel(SigPtWorldInd{(clus)});
    else
        total3DptsPerClus_tmp = 30;
    end
    for threeDPt = 1 : total3DptsPerClus_tmp
%         numel(SigPtWorldInd{clus})
        
        ThreeD_Ind = (SigPtWorldInd{(clus)}(threeDPt));%{clus}

        NViews_3D_init = ThreeD_Pts_Global.pts{ThreeD_Ind}.views;
        
        %% Find the index of the 3D point for each view
        cnt = 1;
        for k = 1 : numel(NViews_3D_init)
            
            
            if numel(find(subsampleSet == NViews_3D_init(k))) == 0
                continue;
            end
            NViews_3D(cnt) = find(subsampleSet == NViews_3D_init(k));
            NView_3D_Index(cnt) = find(ThreeD_ImgStruct_Global.view{NViews_3D_init(k)}.index_3D == ThreeD_Ind); % Find the index of the 3D point selected in list of 3D points seen by the image
            
            pix_Cord(cnt,:) = ThreeD_ImgStruct_Global.view{NViews_3D_init(k)}.pixCord(NView_3D_Index(cnt),:);
            cnt = cnt + 1;
        end
        
        
        %% Send the data per 3D point to cluster and make copies
%         R_curr = rotationmat(temp_Rot_RGB);
        if cnt < 2
            disp 'Here'
            continue;
        end
%         if threeDPt == 164
%             disp 'waitoo'
%         end
        [Copies_data] = make_3D_copies( pix_Cord(1:cnt-1,:), NViews_3D(1:cnt-1), partition);
%         continue;
        numOfLocalClus = numel(Copies_data.copy);
%         if clus == 3
%             disp 'Stopp!!!'
%         end
        %% To identify which 3D copies belong to same 3D point
        mergeCount = mergeCount + 1;
%         mergeCopies{mergeCount} = numOfLocalClus;
%         if numOfLocalClus > 1
%             mergeIncr = mergeIncr + nchoosek(numOfLocalClus,2);
%         end
        
        %% Iterate over the copies ; reject cams not consistent with the Tango track and internally relocalize the track !
        iter_copy_track = 0; % Keeps track of the copies i.e in case of merging iter_copy cannot be changed
        for iter_copy = 1 : numOfLocalClus
            
            if size(Copies_data.copy{iter_copy}.pixCord,1) < 2
%                 disp 'Here'
                continue;
            end
            
            iter_copy_track = iter_copy_track + 1;
            threeD_Cnt_instance = threeD_Cnt_instance + 1;
            iter_local = 1;
%             threeD_vec(:,iter_copy) = Copies_data.copy{iter_copy}.WorldCord; 
            R_iter = rotationmat(Opt_Rot_RGB);
            threeD_vec(:,threeD_Cnt_instance) = R_iter{Copies_data.copy{iter_copy}.NViews(1)}*(inv(K)*Copies_data.copy{iter_copy}.pixCord(1,:)') + Opt_C_RGB(Copies_data.copy{iter_copy}.NViews(1),:)';
            viewIncr = viewIncr + size(Copies_data.copy{iter_copy}.pixCord,1);
            
            % Iterate over the edges
            for edge_iter = 1 : min(1000,size(Copies_data.copy{iter_copy}.pixCord,1))
                        
                pix_curr(iter_local,:) = Copies_data.copy{iter_copy}.pixCord(edge_iter,:);
                NViews_curr(iter_local) = Copies_data.copy{iter_copy}.NViews(edge_iter);
                
                pix_curr_Arr{threeD_Cnt_instance} = pix_curr(1:iter_local,:);
                NViews_curr_Arr{threeD_Cnt_instance} = NViews_curr(1:iter_local);
                iter_local = iter_local + 1;
                
            % end iterating over the edges
            end 
        end
        
        mergeCopies{mergeCount} = iter_copy_track;
        if iter_copy_track > 1
            mergeIncr = mergeIncr + nchoosek(iter_copy_track,2);
        end
        
        
%         disp 'Hello Lez C!'
    end
end


%% BA_Switch_Constraints 
Switch_merge = ones(mergeIncr + viewIncr,1);
C_vec = (reshape(Opt_C_RGB',[1,size(Opt_C_RGB,1)*size(Opt_C_RGB,2)]))';
t_vec = (reshape(Opt_Rot_RGB',[1,size(Opt_Rot_RGB,1)*size(Opt_Rot_RGB,2)]))';
threeD_vec_grp = reshape(threeD_vec,[1,size(threeD_vec,1)*size(threeD_vec,2)])';

param = double([threeD_vec_grp; t_vec; C_vec; Switch_merge]);

[J, size_err] = construct_Jacob_Pattern_LoopClosure_JointOpt(size(Opt_Rot_RGB,1), (NViews_curr_Arr), [1], numel(param), threeD_Cnt_instance, mergeCopies, mergeIncr, viewIncr,numel(Switch_merge)) ;
% [J, viewCnt] = construct_Jacob_Pattern_LoopClosure_JointOpt_working(size(Opt_Rot_RGB,1), (NViews_curr_Arr), [1], numel(param), threeD_Cnt_instance, mergeCopies, numel(Switch_merge)) ;
%% Call lsqnonlin


% v = J1 - J;
% c = find(v > 0);

disp('Begining LsqNonLin')
disp (iter_local)

minimization_options=optimset('LargeScale','on',...
    'Jacobian', 'off', ...
    'Algorithm','Trust-Region-Reflective',...
    'JacobPattern', J, ...
    'Display','iter', ...
    'TolFun',1e-16,...
    'TolX',1e-15,...
    'MaxFunEvals',20000,...
    'MaxIter',200);
%                     'Display','iter', ...
% try
    [OptCam, resNorm, resid] = lsqnonlin(@(x) GlobBA_v4_WithC_LoopClosure_JointOpt(x, size_err, Rot_Tango, cen_Tango, pix_curr_Arr, threeD_Cnt_instance, NViews_curr_Arr, mergeCopies, 0, numel(Switch_merge)), param, [],[],minimization_options);
% catch
    disp 'Whattt'
% end

%% Re-arrange optimization parameters 

Switch_merge = OptCam(end-numel(Switch_merge)+1:end); % Switch variables

OptCam = OptCam(1:end-numel(Switch_merge));
% Reshape into 3*M matrix
param_unpack = reshape( OptCam, [3,size(OptCam,1)/3]); 
param_unpack = param_unpack';


X_3D = param_unpack(1:threeD_Cnt_instance,:); % Collect the 1st Num3DPts rows which are the world co-ordinates localized
threeD_vec = X_3D';%(Iter,:)

param_pose = param_unpack(threeD_Cnt_instance+1:end,:);

%% Pose
t_opt = param_pose(1:size(param_pose,1)/2,:);
C_opt = param_pose(size(param_pose,1)/2+1:end,:);

Opt_Rot_RGB = t_opt;
Opt_C_RGB = C_opt;

% % % %% test run
% % % 
% % % C_vec = (reshape(Opt_C_RGB',[1,size(Opt_C_RGB,1)*size(Opt_C_RGB,2)]))';
% % % t_vec = (reshape(Opt_Rot_RGB',[1,size(Opt_Rot_RGB,1)*size(Opt_Rot_RGB,2)]))';
% % % threeD_vec_grp = reshape(threeD_vec,[1,size(threeD_vec,1)*size(threeD_vec,2)])';
% % % 
% % % param = double([threeD_vec_grp; t_vec; C_vec; Switch_merge]);
% % % Switch_merge = Err_wgt(param, size_err, Rot_Tango, cen_Tango, pix_curr_Arr, threeD_Cnt_instance, NViews_curr_Arr, mergeCopies, 0, numel(Switch_merge));
% % % param = double([threeD_vec_grp; t_vec; C_vec; Switch_merge]);
% % % [J, size_err] = construct_Jacob_Pattern_LoopClosure_JointOpt(size(Opt_Rot_RGB,1), (NViews_curr_Arr), [1], numel(param), threeD_Cnt_instance, mergeCopies, mergeIncr, viewIncr,numel(Switch_merge)) ;
% % % % [J, viewCnt] = construct_Jacob_Pattern_LoopClosure_JointOpt_working(size(Opt_Rot_RGB,1), (NViews_curr_Arr), [1], numel(param), threeD_Cnt_instance, mergeCopies, numel(Switch_merge)) ;
% % % %% Call lsqnonlin
% % % 
% % % 
% % % % v = J1 - J;
% % % % c = find(v > 0);
% % % 
% % % disp('Begining LsqNonLin')
% % % disp (iter_local)
% % % 
% % % minimization_options=optimset('LargeScale','on',...
% % %     'Jacobian', 'off', ...
% % %     'Algorithm','Trust-Region-Reflective',...
% % %     'JacobPattern', J, ...
% % %     'Display','iter', ...
% % %     'TolFun',1e-16,...
% % %     'TolX',1e-15,...
% % %     'MaxFunEvals',20000,...
% % %     'MaxIter',100);
% % % %                     'Display','iter', ...
% % % % try
% % %     [OptCam, resNorm, resid] = lsqnonlin(@(x) GlobBA_v4_WithC_LoopClosure_JointOpt(x, size_err, Rot_Tango, cen_Tango, pix_curr_Arr, threeD_Cnt_instance, NViews_curr_Arr, mergeCopies, 0, numel(Switch_merge)), param, [],[],minimization_options);
% % % % catch
% % %     disp 'Whattt'
% % % % end
% % % 
% % % %% Re-arrange optimization parameters 
% % % 
% % % Switch_merge = OptCam(end-numel(Switch_merge)+1:end); % Switch variables
% % % 
% % % OptCam = OptCam(1:end-numel(Switch_merge));
% % % % Reshape into 3*M matrix
% % % param_unpack = reshape( OptCam, [3,size(OptCam,1)/3]); 
% % % param_unpack = param_unpack';
% % % 
% % % 
% % % X_3D = param_unpack(1:threeD_Cnt_instance,:); % Collect the 1st Num3DPts rows which are the world co-ordinates localized
% % % threeD_vec = X_3D';%(Iter,:)
% % % 
% % % param_pose = param_unpack(threeD_Cnt_instance+1:end,:);
% % % 
% % % %% Pose
% % % t_opt = param_pose(1:size(param_pose,1)/2,:);
% % % C_opt = param_pose(size(param_pose,1)/2+1:end,:);
% % % 
% % % Opt_Rot_RGB = t_opt;
% % % Opt_C_RGB = C_opt;
% % % 

disp 'Done Optimizing : Now generating 3D models !'


end

