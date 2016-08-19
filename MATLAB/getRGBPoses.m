function [ TangoPosesRGB_SS, TangoPosesRGB_AL, tangoTimesRGB ] = getRGBPoses( folder,tangoPose,validFiles )
%GETRGBPOSES Summary of this function goes here
%   Detailed explanation goes here

%% List images
imageFiles = dir([folder filesep '*.jpg']);
imageFiles = cat(1,{imageFiles(:).name})';

TangoPosesRGB_SS = cell(length(imageFiles),1);
TangoPosesRGB_AL = cell(length(imageFiles),1);
tangoTimesRGB = zeros(length(imageFiles),1);

poseTimes = tangoPose(:,5);

if nargin<3
   validFiles = true(length(imageFiles),1);
end

idx = 1:length(imageFiles);
idx = idx(validFiles);

%% Get poses
parfor i=idx
   
   imageFile = imageFiles{i};
   %[~,strFile] = fileparts(pointFile);
   
   % Find image file
   %idxImage = strfind(imageFiles,strFile);
   %idx = cellfun(@(x) ~isempty(x), idxImage);
   
   % Get pose number
   %imageFile = imageFiles{idx};
   [~,strFile] = fileparts(imageFile);
   poseNumber = strsplit(strFile,'_');
   poseNumber = str2num(poseNumber{end}) + 1;
   
   % Get corresponding pose
   time = -double(tangoPose(poseNumber,5));
   Q = double(tangoPose(poseNumber,[4 1 2 3]));
   trans = double(tangoPose(poseNumber,6:8));
   
   % Get transformation (SS)
   T_ss = quaternion2matrix(Q);
   T_ss(1:3,end) = trans;
   
   TangoPosesRGB_SS{i} = T_ss;
   tangoTimesRGB(i) = time;
   
   % Find closest AL pose based on timestamps
   diff_al = abs(poseTimes-time);
   [~,inds] = sort(diff_al,'ascend');
   
   idx = inds(1);
   
   % Get transformation (AL)
   Q = double(tangoPose(idx,[4 1 2 3]));
   trans = double(tangoPose(idx,6:8));
   
   T_al = quaternion2matrix(Q);
   T_al(1:3,end) = trans;
   
   TangoPosesRGB_AL{i} = T_al;   
end

end

