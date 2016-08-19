function txPoints = transformPose2RGB(ind, vtkID, imgID, tangoPose, tangoTimes, tangoPoseCam2Dev, TangopointsStruct, TangoPosesRGB_SS)
       %% Transform the point cloud from point cloud pose to RGB pose.       
        
        ptsTest = TangopointsStruct{ind};
        poseTimes = -tangoPose(:,5); % Pose timestamps

        iN = vtkID+1; % Point cloud index _0000
        j  = imgID+1 ; % Zero-based RGB pose index _00011
        
        time = tangoTimes(ind); % Point cloud timestamp
        
        % Find closest pose based on timestamps
        diff = abs(poseTimes-time);
        [XX,inds] = sort(diff,'ascend');
        idx = inds(1);
        
% % %         posePoints = tangoPose(idx,:);
% % %         poseRGB = tangoPose(j,:);
        % Point Cloud Pose
        
        Q = double(tangoPose(idx,[4 1 2 3]));
        trans = double(tangoPose(idx,6:8));
        posePts = (quaternion2matrix(Q));
        posePts(1:3,end) = trans;
        posePtsProj2W = posePts*tangoPoseCam2Dev;
        
        % RGB pose (Sami)
%         time = -tangoPose(j,5);
%         diff = abs(poseTimes-time);
%         [XX,inds] = sort(diff,'ascend');
%         idx1 = inds(1);
% % % %         
%         Q = double(tangoPose(idx1,[4 1 2 3]));
%         trans = double(tangoPose(idx1,6:8));
%         poseImg = (quaternion2matrix(Q));
%         poseImg(1:3,end) = trans;
        poseImg = TangoPosesRGB_SS{vtkID + 1};
        poseImgProj2W = inv(tangoPoseCam2Dev)*inv(poseImg);

% % %         %% Area Learning mode
% % %         Q = double(tangoPose(j,[4 1 2 3]));
% % %         trans = double(tangoPose(j,6:8));
% % %         poseImg = (quaternion2matrix(Q));
% % %         poseImg(1:3,end) = trans;
% % %         poseImgProj2W = inv(tangoPoseCam2Dev)*inv(poseImg);

        %% Transform point cloud
        for ptCnt = 1 : size(ptsTest,1)

              ptW = posePtsProj2W*[ptsTest(ptCnt,:) 1]';
              tmp = poseImgProj2W*ptW;
              ptsTmp(ptCnt,1:3) = tmp(1:3);
              
        end
        txPoints = ptsTmp(1:size(ptsTest,1),1:3);
end