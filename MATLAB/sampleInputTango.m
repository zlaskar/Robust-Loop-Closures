% matlab script to copy every Nth file from one folder to another folder
% This function samples the Input data so that openMVG need not be run on
% consecutive frame where same features are observed in a relatively less
% displacement


function[sampledFolder] = sampleInputTango(folder, sampleRate)

imgFiles = dir([folder filesep '*.jpg']);
imgFiles = cat(1,{imgFiles(:).name})';

sampledFolder = [folder filesep 'sampledImages'];
mkdir(sampledFolder);

NumImages = size(imgFiles,1); % Total number of Images in the dataset
% subsampleRate = 2;

for i = 1 : sampleRate : NumImages
   copyfile([folder filesep imgFiles{i}], [sampledFolder filesep imgFiles{i}]);       
end

end