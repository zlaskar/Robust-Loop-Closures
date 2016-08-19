%% Program to scale up the sfm reco to Tango scale
%  Author : Zakaria Laskar
%  Date : 02/07/2015
function [scale, T_3DptsPerImage, StdInd] = getScale(MVGSfm_Data, ImageList, camparams, validFilesTango)
clc;
% % % camparams = [1040.47; 1040.63; 634.03700000000003; 365.99900000000002; 0; 0; 0]; %White
disp 'Getting scale >>>>>>>>>>'
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
      0 fx v0;
      0  0  1];

%% Store the image index of those that have pose available in "extrinsics"
valImgToTotalImgList = zeros(size(MVGSfm_Data.views,2),1);


%%
for i = 1 : size(MVGSfm_Data.extrinsics,2)
    
   valImgToTotalImgList(MVGSfm_Data.extrinsics{1,i}.key + 1,1) = i; % ***NOTE
    
end

% % % vldFilesIndx = find(validFilesSOF > 0);

%% Loop over the structure points (3D world co-ordinates)

% Create a index vector to store the increment in index
indexVec = zeros(size(MVGSfm_Data.extrinsics,2));
T_3DptsPerImage.Num3DPts = size(MVGSfm_Data.structure,2);

countLS = 1;
StdArr = zeros(1,size(MVGSfm_Data.structure,2));
counter_StdArr = 0;
for j = 1 : size(MVGSfm_Data.structure,2)

    disp('Processing 3D point : ')
    disp(j)
    x3D = MVGSfm_Data.structure{1,j}.value.X; % 3*1 vec of 3D point in world cord..
    disp(x3D)
    
    counter_StdArr = counter_StdArr + 1;
    imgList = zeros(1,size(MVGSfm_Data.structure{1,j}.value.observations,2));
    counter_ImgList = 0;
    %%Loop over the images that see this world cord..
    for k = 1 : size(MVGSfm_Data.structure{1,j}.value.observations,2)
        
% % %          vtkID_prev = MVGSfm_Data.structure{1,j}.value.observations{1,k}.key;
        viewID = MVGSfm_Data.structure{1,j}.value.observations{1,k}.key;
        imgName = MVGSfm_Data.views{1,viewID + 1}.value.ptr_wrapper.data.filename;
        imgNumStr = textscan( imgName, '%s', 'delimiter', '_');
        imgStr = textscan(imgNumStr{1}{5}, '%s', 'delimiter', '.');
        imgID = str2num(imgStr{1}{1});    % RGB 0 based index
        vtkID = str2num(imgNumStr{1}{4}); % 0-based index
        
        counter_ImgList = counter_ImgList + 1;
        imgList(counter_ImgList) = vtkID;
        
%         curVTK = vtkID + 1;
        
%         if curVTK == 574
%             disp 'Caught u!!'
%         end
%         lasVTK = curVTK;
        
%         if viewID ~= vtkID
% %             disp 'WTF'
%         end
        
%         extrinsicID = valImgToTotalImgList(vtkID + 1 ,1);     % ***NOTE
        extrinsicID = valImgToTotalImgList(viewID + 1 ,1);     % ***NOTE
        % Get the pixel co-ords from MVG
        imgX = (MVGSfm_Data.structure{1,j}.value.observations{1,k}.value.x(1));
        imgY = (MVGSfm_Data.structure{1,j}.value.observations{1,k}.value.x(2));

        % Compute the pixel cords ourselves
        R_MVG = MVGSfm_Data.extrinsics{1,extrinsicID}.value.rotation;
        C_MVG = MVGSfm_Data.extrinsics{1,extrinsicID}.value.center;
        pix =  K*(R_MVG*x3D' + (-R_MVG*C_MVG'));
        
        
% % %         imgX_self = (pix(1)/pix(3));
% % %         imgY_self = (pix(2)/pix(3));
% % %         %% (test)
% % %         x3D_local = R_MVG'*inv(K)*pix + C_MVG';
% % %         
        
        
        %% Store the 3D point seen by the image with Image ID img+1
        indexVec(extrinsicID) = indexVec(extrinsicID) + 1;
% % %         T_3DptsPerImage{extrinsicID}{indexVec(extrinsicID)}{1} = x3D;
        T_3DptsPerImage.view{extrinsicID}.index_3D(indexVec(extrinsicID)) = j;
% % %         T_3DptsPerImage.view{extrinsicID}.pixCord{indexVec(extrinsicID)} = [imgX_self, imgY_self];
        T_3DptsPerImage.view{extrinsicID}.pixCord(indexVec(extrinsicID),:) = pix';

        
% % %         if vtkID+1 == 207
% % %             disp 'Here'
% % %         end
% % %         
        %% If no point cloud available save computation
        %% Process for gettnig scale
%         if validFilesTango(vtkID + 1) == 0 || size(ImageList{valImgToTotalImgList(vtkID + 1)},2) == 0
        try
        if validFilesTango(viewID + 1) == 0 || size(ImageList{valImgToTotalImgList(viewID + 1)},2) == 0
% % %             ImageList{i} = depthMat;
            disp 'No valid points in getScale'
            continue;
        end
        
        catch
            disp ('HoldON')
        end
        
        %% Continue with scale optimization ... :
        R = MVGSfm_Data.extrinsics{1,extrinsicID}.value.rotation; % 3*3 rot matrix
        C = MVGSfm_Data.extrinsics{1,extrinsicID}.value.center;   % 3*1 camera center w.r.t world cord..
                
%         d(countLS) = getTangoProjDepth(round(imgX), round(imgY), ImageList{valImgToTotalImgList(vtkID + 1)}); % ImageList loaded from ''TangoBackProjImg''
        d(countLS) = getTangoProjDepth(round(imgX), round(imgY), ImageList{valImgToTotalImgList(viewID + 1)}); % ImageList loaded from ''TangoBackProjImg''
        u(countLS) = getOrthProj(R, C, x3D, K);
        
% % %         %% Modification (Juho)
% % % %         xcam = [R -R*C']*[x3D 1]';
% % % %         openMVGPtCld{1,extrinsicID}(countLS,:) = xcam(1:3)';



        %%
        countLS = countLS + 1;

    end
    
    StdArr(counter_StdArr) = numel(imgList)*std(imgList);
%     disp '>>> Donee' 
end

% Find the 3D point with mx std dev (assuming that indicates loop closure point)
% [mStd, StdInd] = max(StdArr);
[mStd, StdIndSort] = sort(StdArr, 'descend');

% If more than 30 3D points available choose the first 30 else select all
if numel(StdIndSort) > 1000
    StdInd = StdIndSort(1:1000);
else
    StdInd = StdIndSort(1:numel(StdIndSort));
end
%% Visualization 
%%Include only those points whose depth is available
ind = d > 0;
dLS = d(ind);
uLS = u(ind);

if numel(uLS) > 0
    scale = uLS*dLS'/(uLS*uLS');
else
    scale = 1;
end
% % % ***NOTE                                                     : +1 as MATLAB
                                                                % is 1
                                                                % index and
                                                                % extrinsics
                                                                % are
                                                                % 0-index
end