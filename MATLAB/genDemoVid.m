
function ptCloudScene = genDemoVid(openMVGPointsWorld,R_opt, Opt_C_RGB, RGB4Pts_subsampled, folder)

%% Assign TangoPose
for i = 1 : length(Opt_C_RGB)
    
   TangoPosesWorld{i} =  [R_opt{i} Opt_C_RGB(i,:)'];
    
end
TangoPointsWorld = openMVGPointsWorld;
imgFiles = dir([folder filesep '*.jpg']);
imgFiles = cat(1,{imgFiles(:).name})';
%gridSize = 0.01;
mergeSize = 0.01;

% % % outDir = [folder filesep 'scene'];
% % % [~,~] = mkdir(outDir);

moving = pcdenoise(pointCloud(TangoPointsWorld{1}),'NumNeighbors',16);
ptCloudScene = moving;
% set(gca,'visible','off');
figure('Renderer','opengl');
% imshow(zeros(720,1280,3,'uint8'));
set(gca,'visible','off');
% showPointCloud((TangoPointsWorld{1}), uint8(RGB4Pts_subsampled{1}),'VerticalAxis','Z', 'VerticalAxisDir', 'Up','MarkerSize',16);
% view(10,45);
%zlim([-1.5 1.5]);
%caxis([-1.5 0.5])
% set(gca,'visible','off');
%
fig = gcf;
style = hgexport('factorystyle');
style.Bounds = 'tight';

if ispc
   hgexport(fig,'-clipboard',style,'applystyle', true);
else
   hgexport(fig,'tmp.eps',style,'applystyle', true);
end
drawnow;
% set(gca,'visible','off');
% strFile = [outDir filesep dataSet '_scene_' sprintf('%04d',1) '.png'];
% savePNG(fig,strFile);

allPos = zeros(length(TangoPosesWorld),3);
T = TangoPosesWorld{1};
allPos(1,:) = T(1:3,4);


%% Video obj
Vobj = VideoWriter('Uni_demo.avi');
Vobj.FrameRate = 3;
open(Vobj);
%% Loop through all valid frames
% % % idx = 2:length(TangoPointsWorld);
% % % idx = idx(validFiles(2:end));
for i = 1 : length(Opt_C_RGB)
    
    clf;

   if size(TangoPointsWorld{i},1) < 1
       continue;
   end
    
   ptCloudCurrent = pcdenoise(pointCloud(TangoPointsWorld{i}),'NumNeighbors',16);
   
   %% Use previous moving point cloud as reference.
   %fixed = moving;
   %fixed = ptCloudScene;
   %moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
   %moving = ptCloudCurrent;
   
   % Apply ICP registration.
   %tform = pcregrigid(moving, fixed, 'Metric','pointToPoint','Extrapolate',true,'InlierRatio',1,'Verbose',true);
   
   % Transform the current point cloud to the reference coordinate system
   % defined by the first point cloud.
   %    if i>2
   %       accumTform = affine3d(tform.T * accumTform.T);
   %    else
   %      accumTform = tform;
   %    end
   
   %% Pose
   T = TangoPosesWorld{i};
   allPos(i,:) = T(1:3,4);
   
   %% Neighbor points
   dist = bsxfun(@minus,allPos(i,:),allPos(1:i,:));
   dist = dist.^2;
   dist = sum(dist,2);
   
   %[~,idx] = sort(dist,'ascend');
   %idxl = min(15,length(idx));
   %idx = idx(2:idxl);
   idx = dist<10.^2;
   
   idx_smallMap = dist<100.^2;
   % Pick closest positions
   closePoints = cat(1,TangoPointsWorld{idx});
   closeRGBPts = uint8(cat(1,RGB4Pts_subsampled{idx}));
   
%    closePoints_smallMap = cat(1,TangoPointsWorld{idx_smallMap});
%    closeRGBPts_smallMap = uint8(cat(1,RGB4Pts_subsampled{idx_smallMap}));
   %% Visualization
   
   

   % Visualize the world scene.

   axes('Position',[0 0 1 1]);
   %set(gca,'Position',[0 0 1 1]);
   
   showPointCloud((closePoints), closeRGBPts,'VerticalAxis','Z', 'VerticalAxisDir', 'Up','MarkerSize',16);
   set(gca,'visible','off');
   % Update the world scene.
   %[tform,ptCloudCurrent] = pcregrigid(ptCloudCurrent,ptCloudScene,'Metric','pointToPlane','Verbose',true);
   ptCloudScene = pcmerge(ptCloudScene, ptCloudCurrent, mergeSize);
   
   hold on;
   plotCamera('Location',T(1:3,4),'Orientation',T(1:3,1:3)','Size',0.1,'Opacity',0);
   scatter3(ptCloudCurrent.Location(:,1),ptCloudCurrent.Location(:,2),ptCloudCurrent.Location(:,3),12,'ko','filled');
   scatter3(allPos(idx,1),allPos(idx,2),allPos(idx,3),16,'ro','filled');
   hold off;
   view(-120,120);
  
   % Small Map
%    axes('Position',[0.0 0.75 .25 .25]);
%    %set(gca,'Position',[0 0 1 1]);
%    
%    showPointCloud((closePoints_smallMap), closeRGBPts_smallMap,'VerticalAxis','Z', 'VerticalAxisDir', 'Up','MarkerSize',16);
%    set(gca,'visible','off');
%    % Update the world scene.
%    %[tform,ptCloudCurrent] = pcregrigid(ptCloudCurrent,ptCloudScene,'Metric','pointToPlane','Verbose',true);
% %    ptCloudScene = pcmerge(ptCloudScene, ptCloudCurrent, mergeSize);
%    
%    hold on;
% %    plotCamera('Location',T(1:3,4),'Orientation',T(1:3,1:3)','Size',0.1,'Opacity',0);
% %    scatter3(ptCloudCurrent.Location(:,1),ptCloudCurrent.Location(:,2),ptCloudCurrent.Location(:,3),12,'ko','filled');
% %    scatter3(allPos(:,1),allPos(:,2),allPos(:,3),16,'ro','filled');
%    hold off;
%    view(-90,10);
   
% % %    set(gca,'visible','off');
   axes('Position',[0.75 0.75 .25 .25]);
%    set(gca,'visible','off');
   imPath = [folder '/' imgFiles{i}];
   im = imread(imPath);
%    subplot(3,4,4);
   imshow(im);
   set(gca,'visible','off');
   fig = gcf;
   set(gca,'visible','off');
   if ispc
      hgexport(fig,'-clipboard',style,'applystyle', true);
   else
      hgexport(fig,'tmp.eps',style,'applystyle', true);
   end
%    set(gca,'visible','off');
   drawnow;
%    set(gca,'visible','off');
   F = getframe(fig);
   writeVideo(Vobj,F);
   
   
   
% % %    strFile = [outDir filesep dataSet '_scene_' sprintf('%04d',i) '.png'];
% % %    savePNG(fig,strFile);
end
%% Insert additional images
for j = 1 : 10

    im = imread('data_SC/jannevid/view_top.jpg');
    im = imresize(im,[size(F.cdata,1), size(F.cdata,2)]);
    writeVideo(Vobj, im);    

end
for j = 1 : 10

    im = imread('data_SC/jannevid/topview.jpg');
    im = imresize(im,[size(F.cdata,1), size(F.cdata,2)]);
    writeVideo(Vobj, im);    

end

for j = 1 : 10

    im = imread('data_SC/jannevid/view_front.jpg');
    im = imresize(im,[size(F.cdata,1), size(F.cdata,2)]);
    writeVideo(Vobj, im);    

end

for j = 1 : 10

    im = imread('data_SC/jannevid/map_overlay.jpg');
    im = imresize(im,[size(F.cdata,1), size(F.cdata,2)]);
    writeVideo(Vobj, im);    

end



%% Visualize the entire scene
figure;
imshow(zeros(1024,1024,3,'uint8'));
showPointCloud(ptCloudScene, 'VerticalAxis','Z', 'VerticalAxisDir', 'Up','MarkerSize',12);

hold on;
scatter3(allPos(:,1),allPos(:,2),allPos(:,3),16,'ro','filled');
hold off;

%view(10,45);

fig = gcf;

drawnow;
set(gca,'visible','off');
%% Save MATLAB figure file
% % % strFile = [outDir filesep dataSet '_world.fig'];
% % % savefig(fig,strFile);

%cameratoolbar('Close');
%set(gcf,'ToolBar','none');
%set(gcf,'MenuBar','none');

if ispc
   hgexport(fig,'-clipboard',style,'applystyle', true);
else
   hgexport(fig,'tmp.eps',style,'applystyle', true);
end

%% Save png
% % % strFile = [outDir filesep dataSet '_world.png'];
% % % savePNG(fig,strFile);

%% Switch to topview
view(0,90);

% % % strFile = [outDir filesep dataSet '_world_top.png'];
% % % savePNG(fig,strFile);

end