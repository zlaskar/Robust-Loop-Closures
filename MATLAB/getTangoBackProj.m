function[ImageList, TangoTxPoints, RGBStruct] = getTangoBackProj(openMVGsmf_data, TangopointsStruct, validFilesFromTango, tangoPose, tangoTimes, tangoPoseCam2Dev, camparams, vldFilesIndx, TangoPosesRGB_SS,folder)
    
    disp 'Transforming Points from point cloud Pose to RGB pose and corresponding Depth map'
% % %     vldFilesIndx = find(validFilesFromTango > 0);

    for i = 1:size(openMVGsmf_data.extrinsics,2)
        % % %         disp 'Processing Image Num :   '
        % % %         disp (i)
        %         imgID = openMVGsmf_data.extrinsics{1,i}.key;
        
        % % %         camparams = [1040.47; 1040.63; 634.03700000000003; 365.99900000000002; 0; 0; 0]; %White
        
        % w = 640; h = 360;
        w1 = 1280; h1 = 720;
        % Focal length
        fx1 = camparams(1);
        fy1 = camparams(2);
        
        % Principal point
        u01 = camparams(3);
        v01 = camparams(4);
        
        % Distortion parameters
        k11 = camparams(5);
        k21 = camparams(6);
        k31 = camparams(7);
        
        K1 = [fx1  0 u01;
            0 fy1 v01;
            0  0  1];
        
% % %         depthMat = double(zeros([w1,h1]));
        
        viewID = openMVGsmf_data.extrinsics{1,i}.key;
        
        imgName = openMVGsmf_data.views{1,viewID + 1}.value.ptr_wrapper.data.filename;
        
        % % %         %% Mod  (Pose)
        % % %         vId = openMVGsmf_data.extrinsics{1,i + 1}.key;
        % % %         imgNametmp = openMVGsmf_data.views{1,vId + 1}.value.ptr_wrapper.data.filename
        % % %
        % % %
        
        %%
% % %                 immrgb = imread(imgName);
        
        
        imgNumStr = textscan( imgName, '%s', 'delimiter', '_');
        vtkID = str2num(imgNumStr{1}{4}); % 0-based index
        vtkStr = textscan(imgNumStr{1}{5}, '%s', 'delimiter', '.');
        imgID = str2num(vtkStr{1}{1});
        % If no Point available, continue
        if validFilesFromTango(vtkID + 1) == 0
            % % %             ImageList{i} = depthMat;
            disp 'No valid points in getTangoBackProj'
            TangoTxPoints{i} = [];
            ImageList{i} = [];
            RGBStruct{i} = [];
            continue;
        end
        % else :
        ind = find(vldFilesIndx == vtkID + 1);
        
        pts = transformPose2RGB(ind, vtkID, imgID, tangoPose, tangoTimes, tangoPoseCam2Dev, TangopointsStruct, TangoPosesRGB_SS);
       
        


% Above part shifted to function getDepthMap(...)
        imgNameFull = [folder '/' imgName];
        img = imread(imgNameFull);
        [ImageList{i}, RGBStruct_init, visiblePts] = getDepthMap(pts, img);
% % %         , k11, k21, k31, w1, h1);

%%         
        validPts = find(visiblePts > 0);
        RGBStruct{i} = RGBStruct_init(validPts,:);
        TangoTxPoints{i} = pts(validPts,:);

    end
% % %     
% % %      save Test_clus2 ImageList
% % %      save TxTest_clus2 TangoTxPoints
end


