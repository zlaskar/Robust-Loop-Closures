%% Function to cluster the Adjacency Matrix from openMVG matching stage
%%% Author : Zakaria Laskar
%%% Date   : 22/07/2015

% Input : folder path
%       : Minimum number of Images a cluster should have

% Output: Number of clusters 
%       : folder 'matchFolderNew' that contains sub folders 'clus_Incr_' with
% matches.e.txt files inside them. These files represent the feature
% matches between the images in that cluster and are to be fed to OpenMVG GlobalSfm
% pipeline iteratively

function[clusCnt, Adj, ImageMatchCnt] = clusterOpnMVG(folder, minImgPerClus)


filePath = [folder '/SfM/matches/matches.e.txt'];
fid = fopen(filePath, 'r');
Lines = textscan( fid, '%s', 'delimiter', '\n');
fclose( fid );

imgFiles = dir([folder filesep '*.jpg']);
imgFiles = cat(1,{imgFiles(:).name})';
NumOfClusInit = 0;
NumImages = size(imgFiles,1); % Total number of Images in the dataset

pairNum = 1;
lineNum = 1;
curImgNum = 0;
clusCnt = 0;
ImageMatchCnt = 0;
% Loop over the lines
while lineNum < size(Lines{1},1)
    
    tmp = lineNum;
    
    %% Get the Image Match Pair
    pairIdS = textscan( Lines{1}{lineNum}, '%d', 'delimiter', ' '); % 2*1 list containing pair IDs
    pairs(pairNum,:) = [pairIdS{1}(1)+1 pairIdS{1}(2)+1]; % '+1' as zero index
    ImageMatchCnt = ImageMatchCnt + 1; % increment the match counter 
    pairNum = pairNum + 1;
    pairs(pairNum,:) = [pairIdS{1}(2)+1 pairIdS{1}(1)+1]; % if 0 1 match also store 1 0
    pairNum = pairNum + 1;
    
    %% Get the Number of features that match between the images in the pair to skip that many lines
    lineNum = lineNum + 1;
    featMatchNum = textscan( Lines{1}{lineNum}, '%d');
    lineNum = lineNum + featMatchNum{1}(1) + 1;
    
    
    %% Extras for storing already computed feature matches
    if (pairIdS{1}(1) + 1)  == curImgNum
        ImgCorr{1,pairIdS{1}(1)+1}.endNum = lineNum - 1;
    else
        ImgCorr{1,pairIdS{1}(1)+1}.startNum = tmp;
        curImgNum = pairIdS{1}(1)+1;
        ImgCorr{1,pairIdS{1}(1)+1}.endNum = lineNum - 1;
    end
    
    
end
pairs = double(pairs);

%% Create Adjacency Matrix
% % % A = accumarray(pairs, 1);
Adj = sparse(pairs(:,1), pairs(:,2), 1); % Store it as sparse
Adj(eye(size(Adj))~=0)=1; %% Fill diagonal elements as 1 (faster way possible)

%% Create clusters using CCA
[p,q,r,] = dmperm(Adj);

%% Testing Purpose if clusters are working (working :))))
 clusName = fullfile(folder,'/SfM/matches/clusters');
 [~,~] = mkdir(clusName);
% clusCnt = 0;
% for k = 1 : size(r,2) - 1
%     
%     clus = p(r(k):r(k+1)-1);
%     
%     if size(clus,2) > minImgPerClus
%         clusCnt = clusCnt+1;
%         clusFolderName = fullfile(clusName, sprintf('clus_%03d', clusCnt));
%         [~,~] = mkdir(clusFolderName);
%         
%         for j = 1 : size(clus,2)
%             
%             imgPath = sprintf('%s/%s', folder, imgFiles{clus(j)});
%             %im = imread(imgPath);
%             outImgPath = sprintf('%s/%s', clusFolderName, imgFiles{clus(j)});
%             %imwrite(im, outImgPath);
%             copyfile(imgPath, outImgPath);
%         end
%     end
% end
% 
% 
% %% Create seperate match files
clusCnt = 0;
for k = 1 : size(r,2) - 1
    clus = p(r(k):r(k+1)-1);
    if size(clus,2) > minImgPerClus
        
        clusCnt = clusCnt+1;
        clusFolderName = fullfile(clusName, sprintf('clus_%03d', clusCnt));
        [~,~] = mkdir(clusFolderName);
        
        matchFileName = fullfile(clusFolderName,'matches.e.txt');
        
        fid = fopen(matchFileName, 'w');
        for j = 1 : size(clus,2)
            if clus(j)> size(ImgCorr,2)
                continue;
            elseif size(ImgCorr{1,clus(j)},2) < 1
                continue;
            else
                fprintf( fid, '%s\n', Lines{1}{ImgCorr{1,clus(j)}.startNum : ImgCorr{1,clus(j)}.endNum});
            end
            
        end
        fclose(fid);
    end
end
end
