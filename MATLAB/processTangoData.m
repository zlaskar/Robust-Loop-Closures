%% script
% Author : Sami Huttunen / Zakaria Laskar
% Input :  Provide the dataset path to the variable 'dataPath'
% Output:  .mat file containing pose and other information

%% Define dataset

%dataPath = 'Esa/TangoData_20150902_102939_2079files';
%dataPath = 'samhut/TangoData_20150910_173027_853files';
%dataPath = 'openmvg/TangoData_20151010_185048_3917files';
%dataPath = 'openmvg/TangoData_20151024_203210_1211files';
% dataPath = 'openmvg/uni_CMV_New_2'; % (corrupt dataset)
% dataPath = 'openmvg/University_Oulu';

%dataPath = 'Esa/TangoData_20150901_120806_2019files_Z';
d%ataPath = 'Aalto_interns/TangoData_20160512_111242_701files';
dataPath = 'datasets/TangoData_20160511_211238_651files'


dataSet = strsplit(dataPath,'/');
dataSet = dataSet{end};
folder = ['/research/imag/development/GoogleTango/data/' dataPath];

if ispc
   folder = convertFileNames(folder);
end

outFolder = fullfile(folder,'matlab');

if ~exist(folder,'dir')
   unzip([folder '.zip'],folder);
end

[~,~] = mkdir(outFolder);
outFile = fullfile(outFolder,[dataSet '_matlab.mat']);

%% Import data
[TangoPoints,tangoTimes,tangoPose,tangoPoseCam2Dev,pointFiles,validPointFiles] = vtkImportTangoData(folder);
TangoPoints = TangoPoints(validPointFiles);
tangoTimes = tangoTimes(validPointFiles);
%pointFiles = pointFiles(validPointFiles);
%validPointFiles = true(length(validPointFiles),1);

save(outFile);

%% Map to world coordinates
[TangoPointsWorld_SS,TangoPosesWorld_SS,TangoPointsWorld_AL,TangoPosesWorld_AL] = mapTangoToWorld(TangoPoints,tangoTimes,tangoPose,tangoPoseCam2Dev,folder,pointFiles(validPointFiles));
save(outFile,'TangoPointsWorld_SS','TangoPosesWorld_SS','TangoPointsWorld_AL','TangoPosesWorld_AL','-append');

%% Get RGB poses
[ TangoPosesRGB_SS,TangoPosesRGB_AL,tangoTimesRGB ] = getRGBPoses( folder,tangoPose );
save(outFile,'TangoPosesRGB_SS','TangoPosesRGB_AL','tangoTimesRGB','-append');

%% Show trajectories
[ posesLearning,posesService ] = showTrajectories( tangoPose );
save(outFile,'posesLearning','posesService','-append');

%% Create depth maps
%depthMap(TangoPoints,folder,dataSet,validFiles);

%% Register point clouds
%ptCloudScene = registerPoints(TangoPointsWorld_SS,TangoPosesWorld_SS,folder,dataSet,validFiles);

%% Create video
%createVideo(folder,dataSet);

%% Sampling
sampleRate = 3; % subsample the image list to preserve computation in openMVG
[sampledFolder] = sampleInputTango(folder, sampleRate);

%%
outSampleFolder = fullfile(folder,'matlab','optim');
[~,~] = mkdir(outSampleFolder);
outSampleFile = fullfile(outSampleFolder,[dataSet '_matlab_s' num2str(sampleRate) '.mat']);
save(outSampleFile,'sampledFolder','sampleRate');

%% openMVG + Pose-graph optimization (Modified ~Z)
%[matches.e.txt] = Do image matching in openMVG
openMVG_match(sampledFolder);

minImgPerClus = 3;
[NumOfClusInit, Adj, ImageMatchCnt] = clusterOpnMVG(sampledFolder, minImgPerClus);
save(outSampleFile,'minImgPerClus','NumOfClusInit','-append');

%% openMVG Global SfM on clusters
% Now pass the NumOfClusInit matches.e.txt files to globalSFM stored in
% sampledfolder/matchNewFolder/clus_%d/matches.e.txt. Output shall be
% sampledfolder/SfM/ will contain reconstruction_%d folders. Some will have
% sfm_data.json files. As such the actual number of clusters shall be less.
openMVG_global(sampledFolder, NumOfClusInit);

%% Scaling and transforms
% Iterate over the json files and do scaling and do pose transforms
[NumOfClus,RGB4Pts, RGB4Pts_AL, SigPtWorldInd, ThreeD_ImgStruct_Global, total3DLocalToClusIncrem, imgFiles, R_RGB_Tango, t_RGB_Tango, R_RGB_Tango_AL, t_RGB_Tango_AL ,TangoTxPoints, TangoTxPoints_AL,vldFilesIndx]= ChainTangoTxv2( TangoPoints, validPointFiles, tangoPose, tangoTimes, tangoPoseCam2Dev, folder, TangoPosesRGB_SS, TangoPosesRGB_AL, NumOfClusInit);
% save(outSampleFile,'NumOfClus','RGB4Pts','RGB4Pts_AL','SigPtWorldInd','ThreeD_ImgStruct_Global','total3DLocalToClusIncrem','imgFiles','R_RGB_Tango','t_RGB_Tango','R_RGB_Tango_AL','t_RGB_Tango_AL','TangoTxPoints','TangoTxPoints_AL','vldFilesIndx','-append');

%% Optimization
ThreeD_Pts_Global = joinClustersVer2(ThreeD_ImgStruct_Global,total3DLocalToClusIncrem, imgFiles); % Useless for later
save(outSampleFile,'ThreeD_Pts_Global','-append');

% main script that does optimization
partition = 10; % Gap between images to make clusters for a 3D point
minNumViewsPer3Dpt = 2; % minimum views a 3D point must be seen by

% Old version (slow)
%[tx_sub, Opt_Rot_RGB, Opt_C_RGB, t_subsampled_RGB, C_RGB, t_subsampled_RGB_AL, C_RGB_AL, RGB4Pts_subsampled, RGB4Pts_AL_subsampled] = main_final_LC_copy(ThreeD_ImgStruct_Global, ThreeD_Pts_Global, R_RGB_Tango, t_RGB_Tango, SigPtWorldInd, TangoTxPoints, R_RGB_Tango_AL, t_RGB_Tango_AL, TangoTxPoints_AL, RGB4Pts, RGB4Pts_AL,sampleRate, NumOfClus, partition, minNumViewsPer3Dpt);

% New version (fast, with Switch constraints)
[Switch_merge, tx_sub, tx_sub_AL, Opt_Rot_RGB, Opt_C_RGB, t_subsampled_RGB, C_RGB, t_subsampled_RGB_AL, C_RGB_AL, RGB4Pts_subsampled, RGB4Pts_AL_subsampled] = main_SC(ThreeD_ImgStruct_Global, ThreeD_Pts_Global, R_RGB_Tango, t_RGB_Tango, SigPtWorldInd, TangoTxPoints, R_RGB_Tango_AL, t_RGB_Tango_AL, TangoTxPoints_AL, RGB4Pts, RGB4Pts_AL,sampleRate, NumOfClus, partition, minNumViewsPer3Dpt);
save(outSampleFile,'partition','minNumViewsPer3Dpt','Switch_merge','tx_sub', 'tx_sub_AL','Opt_Rot_RGB','Opt_C_RGB','t_subsampled_RGB','C_RGB','t_subsampled_RGB_AL','C_RGB_AL','RGB4Pts_subsampled','RGB4Pts_AL_subsampled','-append');

% Plot results
mapGen(tx_sub,tx_sub_AL,Opt_Rot_RGB, Opt_C_RGB, t_subsampled_RGB, C_RGB, t_subsampled_RGB_AL, C_RGB_AL, RGB4Pts_subsampled, RGB4Pts_AL_subsampled);
