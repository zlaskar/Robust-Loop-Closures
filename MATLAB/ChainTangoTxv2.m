%% Function that finds the correct timeline based transformation for each Image/ Cluster to the 1st cluster
% Author : Zakaria Laskar
% Date : 29/07/2015
% Final version

% Input : 

% TangoPointsSOF : point cloud information for each valid image
% validFilesSOF  : valid files i.e images that have corresponding point
% cloud data
% tangoPoseSOF   : pose information available from Tango
% tangoPoseCam2Dev: transformation matrix from camera to device
% co-ordinates
% camparams      : camera parameters of Google Tango camera
% folder         : folder specifying the path 
% OutPut : 
% bestTxInd : Is a structure in the form [Img1, Img2]; if the current image
% belongs to an OpenMVG cluster, Img1 is the reference Image in the cluster
% the transformation to which is calculated later; else Img1 refers to the
% current Image itself. Img2 is the reference Image in the complete
% timeline that offers closest transformation to the 1st cluster.

% sfmData1,
function [NumOfClus,TangoRGB4PtCld, TangoRGB4PtCld_AL, SigPtWorldInd, ThreeD_ImgStruct_Global, total3DLocalToClusIncrem, imgFiles, R_RGB_Tango, t_RGB_Tango, R_RGB_Tango_AL, t_RGB_Tango_AL ,TangoTxPoints, TangoTxPoints_AL,vldFilesIndx, sfmPoses_Rot, sfmPoses_C]= ChainTangoTxv2( TangoPointS, validFileS, tangoPoseS, tangoTimeS, tangoPoseCam2Dev, folder, TangoPosesRGB_SS, TangoPosesRGB_AL, NumOfClusInit)
% [bestTxInd, total3DLocalToClusIncrem, imgFiles, R_Tango_Conv, R_RGB_Tango, t_Tango_Conv, t_RGB_Tango, TangoTxPoints, clusInd, vldFilesIndx]
camparams = [1040.47; 1040.63; 634.03700000000003; 365.99900000000002; 0; 0; 0]; %White

% % % folder = 'C:/Users/zlaskar/Documents/MATLAB/data/TangoData_20150606_204155_2253files';
%% Read the Image Files
imgFiles = dir([folder filesep '*.jpg']);
imgFiles = cat(1,{imgFiles(:).name})';
NumImages = size(imgFiles,1); % User defined
vldFilesIndx = find(validFileS > 0); % find the indices that are valid (so that length is now = to TangoPointsSOF)
% poseTimes = tangoPoseS(:,5); % Pose timestamps
% Init variables
clusArr = 1000*ones(1,NumImages); % Stores the cluster number for each image
% NumOfClus = 1;                    % The number of clusters to read
addpath('../JSON/')

%Init Poses
sfmPoses_Rot = cell(size(TangoPosesRGB_SS));
sfmPoses_C = cell(size(TangoPosesRGB_SS));

total3DLocalToClusIncrem = 0;
NumOfClus = 0;
%% Loop over the OpenMVG clusters
for clusNumInit = 1:NumOfClusInit
    
        disp 'Processing Cluster Number'
        disp (clusNumInit)
               
%         if clusNum > 1
    try
            jsonFilePath = sprintf('%s/sampledImages/SfM/reconstruction/clusters/clus_%03d/sfm_data.json', folder,clusNumInit);
% % %     /clusters/clus_%03d
        sfmData = loadjson(jsonFilePath);
        if numel(sfmData.extrinsics) == 0
            continue;
        end
        NumOfClus = NumOfClus + 1;
    catch  
% %         sfmData = [];
        continue;
    end
       
%         sfmData1{clusNum} = sfmData;

%         sfmData = sfmData1{(clusNumInit)};
        
        % Get the depth map and the point cloud transformed to the RGB pose
        % from point cloud pose
        [ImageList, TxPts, RGB4PtCld] = getTangoBackProj(sfmData,TangoPointS, validFileS, tangoPoseS, tangoTimeS, tangoPoseCam2Dev, camparams, vldFilesIndx, TangoPosesRGB_SS, folder);
        
        % Get the scale to bring the RGB and Tango Point cloud to same
        % scale
        [scale, ThreeD_ImgStruct, index_3D] = getScale(sfmData, ImageList, camparams, validFileS);
        
        % Store the 3D point in the global context
        SigPtWorldInd{NumOfClus} = index_3D + total3DLocalToClusIncrem ;
        
        boundCnt = 0; % variable to store the index of the boundaries of each cluster
        %% Loop over the images in the cluster
        
        for i = 1:size(sfmData.extrinsics,2)
            
            disp (i)
                        
            %% Collect sfm Pose
            R = sfmData.extrinsics{1,i}.value.rotation;
            C = scale*sfmData.extrinsics{1,i}.value.center;
            
            %% Convert to Tango Co-ordinate system (not the origin)
            
            R_Tx_Tango = [0 -1 0;1 0 0;0 0 1]*R';
            C_Tx_Tango = [0 -1 0;1 0 0;0 0 1]*C';
            
            %% Find corresponding Tango camera pose from Image Name
            
            viewID = sfmData.extrinsics{1,i}.key;
            
            imgName = sfmData.views{1,viewID + 1}.value.ptr_wrapper.data.filename;
            imgNumStr = textscan( imgName, '%s', 'delimiter', '_');
            imgStr = textscan(imgNumStr{1}{5}, '%s', 'delimiter', '.');
            imgID = str2num(imgStr{1}{1});
            vtkID = str2num(imgNumStr{1}{4}); % 0-based index
            
            
            %% Store a few arrays : 
            % clusArr : Stores the cluster index for each Image
           
            clusArr(1, vtkID + 1) = NumOfClus;  % Assign the cluster number to the image in array
            % Assign poses from openMVG to struct
            sfmPoses_Rot{vtkID + 1} = R_Tx_Tango;
            sfmPoses_C{vtkID + 1} = C_Tx_Tango;
            %% Stores the transformed points in a global structure
            if validFileS(vtkID + 1) == 0
                TangoTxPoints{vtkID+1} = [];
                TangoRGB4PtCld{vtkID+1} = [];
            else
                TangoTxPoints{vtkID+1} = TxPts{i};
                TangoRGB4PtCld{vtkID+1} = RGB4PtCld{i};
            end
% % %             if size()
            for j = 1:size(ThreeD_ImgStruct.view{i}.index_3D,2)
%                 disp (j)
                ThreeD_ImgStruct_Global.view{vtkID+1}.index_3D(j) = ThreeD_ImgStruct.view{i}.index_3D(j) + total3DLocalToClusIncrem; % Increment the 3D index from local sfmData cluster to global indexing
            end
            ThreeD_ImgStruct_Global.view{vtkID+1}.pixCord = ThreeD_ImgStruct.view{i}.pixCord.*scale;
            ThreeD_ImgStruct_Global.view{vtkID+1}.scale = scale;
            
            
            

            %% Device pose at this instanct of RGB capture
%             time = -tangoPoseS(imgID+1,5);
%             diff = abs(poseTimes-time);
%             [XX,inds] = sort(diff,'ascend');
%             idx1 = inds(1);
% % % %             
% % % %             
%             Q = double(tangoPoseS( idx1 ,[4 1 2 3]));
%             trans = double(tangoPoseS( idx1 ,6:8));
%             poseImg = (quaternion2matrix(Q));
%             poseImg(1:3,end) = trans;
            
            poseImg = TangoPosesRGB_SS{vtkID + 1};
            
% % %             %% Area Learning mode
% % %             Q = double(tangoPoseS( imgID+1 ,[4 1 2 3]));
% % %             trans = double(tangoPoseS( imgID+1 ,6:8));
% % %             poseImg = (quaternion2matrix(Q));
% % %             poseImg(1:3,end) = trans;
% % %             
            
            %% Tango Camera Pose (convert pose to rgb pose)
            R_RGB_Tango{vtkID+1}  = poseImg(1:3,1:3) * tangoPoseCam2Dev(1:3,1:3);
            t_RGB_Tango{vtkID+1}  = poseImg(1:3,end) + poseImg(1:3,1:3)* tangoPoseCam2Dev(1:3,end);
            
        end
        
        disp 'Donee'
        %% Update the total 3D structure counter
        
        total3DLocalToClusIncrem = total3DLocalToClusIncrem + ThreeD_ImgStruct.Num3DPts;
        
        
        %% Find transformation to cluster 1 through nearest neighbour cluster 
%         sizeOfClus = size(clusBoundInd{clusNum},2);
%         if clusNum > 1 % Do for all cluster except 1st, which is the reference cluster
%             
%             %% Compute the nearest neigbor which belongs to a
%             clus = 1; % Stores the cluster associated with the min transformation
%             M = 100000; % very large number
%             
%             %% create a N*M matrix, where N is the number of images in present cluster i, M is the number of images in cluster number j, j~=i
%             for j = 1 : clusNum-1
%                 dist = zeros(sizeOfClus, size(clusBoundInd{j},2));
%                 for k = 1 : sizeOfClus
%                     for h = 1 : size(clusBoundInd{j},2)
%                         
%                         dist(k,h) = abs(clusBoundInd{clusNum}(k) - clusBoundInd{j}(h));
%                         
%                     end
%                 end
%                 [tmp, I] = min(dist(:));
%                 %% If current min is less than previous min replace >>
%                 if tmp < M
%                     M = tmp;
%                     [I_row, I_col] = ind2sub(size(dist), I); % 
%                     clus = j;
%                     
%                 end
%                 
%             end
%             
%             %% Assign the transformation link to all the images in the current cluster
%             for h = 1 : size(clusInd{clusNum},2)
%                 
% % % %                 bestTxInd{clusInd{clusNum}(h)} = [clusBoundInd{clusNum}(I_row), clusBoundInd{clus}(I_col)];
%                 bestTxInd{clusInd{clusNum}(h)} = [clusBoundInd{clusNum}(1), clusBoundInd{clus}(1)];
%             end
%             
%         else % For cluster number 1
%             
%             for h = 1 : size(clusInd{clusNum},2)
%                 
%                 bestTxInd{clusInd{clusNum}(h)} = [clusBoundInd{clusNum}(1) 1];
%                 
%             end
%             
%         end
% % %     end
end

% save Sfm_Oulu_Univ_Final sfmData1


% clusNum = NumOfClus; % init clusNum for non clustered Images
remImagesInd = find(clusArr>NumOfClusInit); % Images that have not been clustered by OpenMVG


%% Loop over the Images
for i = 1 : size(remImagesInd,2)

    %% Get the Tango Information
    imgNumStr = textscan( imgFiles{remImagesInd(i)}, '%s', 'delimiter', '_');
    imgStr = textscan(imgNumStr{1}{5}, '%s', 'delimiter', '.');
    imgID = str2num(imgStr{1}{1});
    vtkID = str2num(imgNumStr{1}{4}); % 0-based index
    
%     clusNum = clusNum + 1; % Increment the cluster num (now each Image is given a unique cluster num with cardinality = 1)
%     clusArr(1,vtkID + 1) = clusNum;  % Assign the cluster number to the image in array
%     clusInd{clusNum}(1) = vtkID+1;
    
    ThreeD_ImgStruct_Global.view{vtkID+1} = [];
    if validFileS(vtkID + 1) == 0 % If no point cloud available
        R_RGB_Tango{vtkID+1}  = R_RGB_Tango{vtkID};
        t_RGB_Tango{vtkID+1}  = t_RGB_Tango{vtkID};
        TangoTxPoints{vtkID+1} = [];
        TangoRGB4PtCld{vtkID+1} = [];
        
        %% Compute the nearest neigbor which belongs to a
%         M = 100000; % very large number
%         clus = 1; % Stores the cluster associated with the min transformation
%         for j = 1 : clusNum
%             dist = zeros(sizeOfClus, size(clusBoundInd{j},2));
%             for k = 1 : sizeOfClus
%                 for h = 1 : size(clusBoundInd{j},2)
%                     
%                     dist(k,h) = abs(vtkID+1 - clusBoundInd{j}(h));
%                     
%                 end
%             end
%             [tmp, I] = min(dist(:));
%             if tmp < M
%                 M = tmp;
%                 [I_row, I_col] = ind2sub(size(dist), I);
%                 clus = j;
%             end
%         end
%         
% % % %         bestTxInd{vtkID+1} = [vtkID+1, clusBoundInd{clus}(I_col)];
%         bestTxInd{vtkID+1} = [vtkID+1, vtkID+1];
%         
%         R_Tango_Conv{vtkID+1} = [];
%         t_Tango_Conv{vtkID+1} = [];
        disp 'No valid points in ChainTangoTx'
        continue;
    end
    
    %% If point cloud avaible do : 
    
% % %     clusNum = clusNum + 1; % Increment the cluster num (now each Image is given a unique cluster num with cardinality = 1)
% % %     clusArr(1,vtkID + 1) = clusNum;  % Assign the cluster number to the image in array
% % %     clusInd{clusNum}(1) = vtkID+1;
    
   
    ind = find(vldFilesIndx == vtkID + 1); % find the point cloud index
    % Get the point cloud transformed to the RGB pose
    
    TangoTxPoints_init = transformPose2RGB(ind, vtkID, imgID, tangoPoseS, tangoTimeS, tangoPoseCam2Dev, TangoPointS, TangoPosesRGB_SS);
    
    imgNameFull = [folder '/' imgFiles{vtkID+1}];
    img = imread(imgNameFull);
    [D, TangoRGB4PtCld_init, visiblePts] = getDepthMap_NonMVG(TangoTxPoints_init, img);
    
    validPts = find(visiblePts > 0);
    TangoRGB4PtCld{vtkID+1} = TangoRGB4PtCld_init(validPts,:);
    TangoTxPoints{vtkID + 1} = TangoTxPoints_init(validPts,:);
    %% Device pose at this instanct of RGB capture
%     time = -tangoPoseS(imgID+1,5);
%     diff = abs(poseTimes-time);
%     [XX,inds] = sort(diff,'ascend');
%     idx1 = inds(1);
% % % %     
%     
%     Q = double(tangoPoseS( idx1 ,[4 1 2 3]));
%     trans = double(tangoPoseS( idx1 ,6:8));
%     poseImg = (quaternion2matrix(Q));
%     poseImg(1:3,end) = trans;
% % %     
      poseImg = TangoPosesRGB_SS{vtkID + 1};
% % %     %% Area Learning mode
% % %     Q = double(tangoPoseS( imgID+1 ,[4 1 2 3]));
% % %     trans = double(tangoPoseS( imgID+1 ,6:8));
% % %     poseImg = (quaternion2matrix(Q));
% % %     poseImg(1:3,end) = trans;
    
    %% Tango Camera Pose
    R_RGB_Tango{vtkID+1}  = poseImg(1:3,1:3) * tangoPoseCam2Dev(1:3,1:3);
    t_RGB_Tango{vtkID+1}  = poseImg(1:3,end) + poseImg(1:3,1:3)* tangoPoseCam2Dev(1:3,end);
%     sizeOfClus = 1;
%     %% Compute the nearest neigbor which belongs to a
%     M = 100000; % very large number
%     clus = 1; % Stores the cluster associated with the min transformation 
%     for j = 1 : clusNum
%         dist = zeros(sizeOfClus, size(clusBoundInd{j},2));
%         for k = 1 : sizeOfClus
%             for h = 1 : size(clusBoundInd{j},2)
%                 
%                         dist(k,h) = abs(vtkID+1 - clusBoundInd{j}(h));
% 
%             end
%         end
%         [tmp, I] = min(dist(:));
%         if tmp < M
%             M = tmp;
%             [I_row, I_col] = ind2sub(size(dist), I);
%             clus = j;
%         end
%     end
%          
% % % %     bestTxInd{vtkID+1} = [vtkID+1, clusBoundInd{clus}(I_col)];
%     bestTxInd{vtkID+1} = [vtkID+1, vtkID+1];
%     R_Tango_Conv{vtkID+1} = [];
%     t_Tango_Conv{vtkID+1} = [];

end
    
%% Get Area Learning Version

for i = 1 : NumImages

    if validFileS(i) == 0
        % % %             ImageList{i} = depthMat;
        disp 'No valid points in getTangoBackProj'
        
        R_RGB_Tango_AL{i}  = R_RGB_Tango{i-1};
        t_RGB_Tango_AL{i}  = t_RGB_Tango{i-1};
        TangoTxPoints_AL{i} = [];
        TangoRGB4PtCld_AL{i} = [];
        continue;
    end
    % else :
    ind = find(vldFilesIndx == i);
    
%     pts = transformPose2RGB(ind, i, 0, tangoPoseS, tangoTimeS, tangoPoseCam2Dev, TangoPointS, TangoPosesRGB_AL);
    
    TangoTxPoints_AL_init = transformPose2RGB(ind, i-1, 0, tangoPoseS, -tangoTimeS, tangoPoseCam2Dev, TangoPointS, TangoPosesRGB_AL);
    imgNameFull = [folder '/' imgFiles{i}];
    img = imread(imgNameFull);
    [ D,TangoRGB4PtCld_AL_init, visiblePts] = getDepthMap_NonMVG(TangoTxPoints_AL_init, img);
    poseImg = TangoPosesRGB_AL{i};
    
    validPts = find(visiblePts > 0);
    TangoRGB4PtCld_AL{i} = TangoRGB4PtCld_AL_init(validPts,:);
    TangoTxPoints_AL{i} = TangoTxPoints_AL_init(validPts,:);
    
    %% Tango Camera Pose
    R_RGB_Tango_AL{i}  = poseImg(1:3,1:3) * tangoPoseCam2Dev(1:3,1:3);
    t_RGB_Tango_AL{i}  = poseImg(1:3,end) + poseImg(1:3,1:3)* tangoPoseCam2Dev(1:3,end);
end







% % % save newwwv2 R_Tango_Conv R_RGB_Tango t_Tango_Conv t_RGB_Tango TangoTxPoints clusArr clusInd clusBoundInd  
disp 'Finished'
% save Cluster_based_opt clusBoundInd remImagesInd
% save Proj_Depth_test ThreeD_ImgStruct_Global SigPtWorldInd
end


