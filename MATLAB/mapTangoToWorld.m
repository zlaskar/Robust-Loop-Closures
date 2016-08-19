function [ TangoPointsWorld_SS,TangoPosesWorld_SS,TangoPointsWorld_AL,TangoPosesWorld_AL ] = mapTangoToWorld( TangoPoints,tangoTimes,tangoPose,tangoPoseCam2Dev,folder,pointFiles,validFiles )
%MAPTANGOTOWORLD Summary of this function goes here
%   Detailed explanation goes here

%% Map to world coordinates
TangoPointsWorld_SS = cell(length(TangoPoints),1);
TangoPosesWorld_SS = cell(length(TangoPoints),1);
TangoPointsWorld_AL = cell(length(TangoPoints),1);
TangoPosesWorld_AL = cell(length(TangoPoints),1);

poseTimes = tangoPose(:,5);

if nargin<7
   validFiles = true(length(TangoPoints),1);
end

%h = figure('Renderer','opengl','Color',[1 1 1]);
%style = hgexport('factorystyle');
%style.Bounds = 'tight';

%% Create output folder
strOutput = [folder filesep 'ply'];
[~,~] = mkdir(strOutput);
strOutputWorld = [strOutput filesep 'world'];
[~,~] = mkdir(strOutputWorld);

idx = 1:length(TangoPoints);
idx = idx(validFiles);

parfor i=idx

   points = double([TangoPoints{i} ones(size(TangoPoints{i},1),1)]);
   time = tangoTimes(i);
   
   % Find closest pose based on timestamps
   diff_ss = abs(-poseTimes-time);
   [~,inds] = sort(diff_ss,'ascend');
   
   idx = inds(1);
   
   Q = double(tangoPose(idx,[4 1 2 3]));
   trans = double(tangoPose(idx,6:8));
   
   T_ss = quaternion2matrix(Q);
   T_ss(1:3,end) = trans;
   
   pointsWorld_ss = T_ss*double(tangoPoseCam2Dev)*points';
   pointsWorld_ss = pointsWorld_ss';
   
   TangoPointsWorld_SS{i} = pointsWorld_ss(:,1:3);
   TangoPosesWorld_SS{i} = T_ss;
   
   % Find closest pose based on timestamps
   diff_ss = abs(-poseTimes-time);
   [~,inds] = sort(diff_ss,'ascend');
   
   idx = inds(1);
   
   Q = double(tangoPose(idx,[4 1 2 3]));
   trans = double(tangoPose(idx,6:8));
   
   T_ss = quaternion2matrix(Q);
   T_ss(1:3,end) = trans;
   
   pointsWorld_ss = T_ss*double(tangoPoseCam2Dev)*points';
   pointsWorld_ss = pointsWorld_ss';
   
   TangoPointsWorld_SS{i} = pointsWorld_ss(:,1:3);
   TangoPosesWorld_SS{i} = T_ss;
   
   % Find closest pose based on timestamps
   diff_al = abs(poseTimes-time);
   [~,inds] = sort(diff_al,'ascend');
   
   idx = inds(1);
   
   Q = double(tangoPose(idx,[4 1 2 3]));
   trans = double(tangoPose(idx,6:8));
   
   T_al = quaternion2matrix(Q);
   T_al(1:3,end) = trans;
   
   pointsWorld_al = T_al*double(tangoPoseCam2Dev)*points';
   pointsWorld_al = pointsWorld_al';
   
   TangoPointsWorld_AL{i} = pointsWorld_al(:,1:3);
   TangoPosesWorld_AL{i} = T_al;
   
   %showPointCloud(pointCloud(TangoPoints{i}),'VerticalAxis','Y','VerticalAxisDir','Down');
   
   %xlim([-1.75 1.75]);
   %ylim([-1.75 1.75]);

   %view(0,-90);
         
   %T = get(gca,'tightinset');
   %set(gca,'position',[T(1) T(2) 1-T(1)-T(3) 1-T(2)-T(4)]);
   %set(gca,'position',[0.05 0.05 0.95 0.95]);
   
   %fig = gcf;
   %hgexport(h,'tmp.eps',style,'applystyle', true);
   
   %drawnow limitrate;
   
   [~,strFile,~] = fileparts(pointFiles{i});
   
   strPLY = [strOutput filesep strFile '.ply'];
   pcwrite(pointCloud(TangoPoints{i}),strPLY,'PLYFormat','binary');
   
   strPLY = [strOutputWorld filesep strFile '_world_SS.ply'];
   pcwrite(pointCloud(TangoPointsWorld_SS{i}),strPLY,'PLYFormat','binary');
   
   strPLY = [strOutputWorld filesep strFile '_world_AL.ply'];
   pcwrite(pointCloud(TangoPointsWorld_AL{i}),strPLY,'PLYFormat','binary');
   
end

%% All raw points
ptCloudAll_ss = pointCloud(cat(1,TangoPointsWorld_SS{:}));
%figure('Name','All points','Renderer','opengl'); showPointCloud(ptCloudAll);
%xlabel('X');
%ylabel('Y');
%zlabel('Z');

[~,strFile,~] = fileparts(pointFiles{1});
strFile = strFile(1:end-5);
strPLY = [strOutputWorld filesep strFile '_all_SS.ply'];
pcwrite(ptCloudAll_ss,strPLY,'PLYFormat','binary');

ptCloudAll_al = pointCloud(cat(1,TangoPointsWorld_AL{:}));
strPLY = [strOutputWorld filesep strFile '_all_AL.ply'];
pcwrite(ptCloudAll_al,strPLY,'PLYFormat','binary');

%% DOwnsampled cloud
gridStep = 0.02;
ptCloudAllFilter_ss = pcdownsample(ptCloudAll_ss,'gridAverage',gridStep);
h_ss = figure('Name','Filtered points (SS)','Renderer','opengl','Color','w');
imshow(zeros(800,800,3,'uint8'));
showPointCloud(ptCloudAllFilter_ss);
xlabel('X');
ylabel('Y');
zlabel('Z');

strPLY = [strOutputWorld filesep strFile '_all_downsample_SS.ply'];
pcwrite(ptCloudAllFilter_ss,strPLY,'PLYFormat','binary');

view(0,90);

strPNG = [strOutputWorld filesep strFile '_all_downsample_SS.png'];
savePNG(h_ss,strPNG);

ptCloudAllFilter_al = pcdownsample(ptCloudAll_al,'gridAverage',gridStep);
h_al = figure('Name','Filtered points (AL)','Renderer','opengl','Color','w');
imshow(zeros(800,800,3,'uint8'));
showPointCloud(ptCloudAllFilter_al);
xlabel('X');
ylabel('Y');
zlabel('Z');

strPLY = [strOutputWorld filesep strFile '_all_downsample_AL.ply'];
pcwrite(ptCloudAllFilter_al,strPLY,'PLYFormat','binary');

view(0,90);

strPNG = [strOutputWorld filesep strFile '_all_downsample_AL.png'];
savePNG(h_al,strPNG);

%savefig(h_ss,[folder filesep 'matlab' filesep strFile '_all_denoise_SS.fig']);
%savefig(h_al,[folder filesep 'matlab' filesep strFile '_all_denoise_AL.fig']);

end

