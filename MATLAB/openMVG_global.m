function openMVG_global(folder,numberOfClusters)

%% Replace matches.e.txt
matchesPath = [folder filesep 'SfM' filesep 'matches'];
matchesFile = [matchesPath filesep 'matches.e.txt'];
matchesFullFile = [matchesPath filesep 'matches_full.e.txt'];

if ~exist(matchesFullFile,'file')
 movefile(matchesFile,matchesFullFile);
end

reconstructionPath = [folder filesep 'SfM' filesep 'reconstruction' filesep 'clusters'];
[~,~] = mkdir(reconstructionPath);

for i=1:numberOfClusters
   %% Get cluster
   cluster = sprintf('clus_%03d', i);
   
   matchesClusterFile = [matchesPath filesep 'clusters' filesep cluster filesep 'matches.e.txt'];
   copyfile(matchesClusterFile,matchesFile);

   %% Run script
   cmd = ['sh ./global_cse-cn0011.sh ' folder ' ' cluster];
   system(cmd, '-echo');
end

end