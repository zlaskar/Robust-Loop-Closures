
function [TangoPoints,tangoTimes,tangoPose,tangoPoseCam2Dev,pointFiles,validFiles] = vtkImportTangoData(folder)

%% Read point clouds
vtkFiles = dir([folder filesep '*.vtk']);
vtkFiles = cat(1,{vtkFiles(:).name})';
idx = true(length(vtkFiles),1);
idxPose = ~cellfun(@(x) isempty(x),strfind(vtkFiles,'_poses.vtk'));
idx(idxPose) = false; 

pointFiles = vtkFiles(idx);

TangoPoints = cell(length(pointFiles),1);
tangoTimes = zeros(length(pointFiles),1,'single');

for i=1:length(pointFiles)
   [TangoPoints{i},tangoTimes(i)] = vtkImportPoints(fullfile(folder,pointFiles{i}));
end

validFiles = ~isnan(tangoTimes);

%% Read pose
poseFile = vtkFiles{idxPose};

[tangoPose,tangoPoseCam2Dev] = vtkImportPose(fullfile(folder,poseFile));
tangoPoseCam2Dev = reshape(tangoPoseCam2Dev,4,4);


end