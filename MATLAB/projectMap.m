
%mapFile = './../../../data/maps/tietotalo_3rdfloor_bw.png';
mapFile = './../../../data/maps/university_1stfloor.png';
mapImage = imread(mapFile);

scale = 50;

%% Interpolate
% points = cat(1,TangoPointsWorld_AL{:});
%points = alignPointCloud(points);
points = xyz;

XYZ = bsxfun(@minus,points,[min(points(:,1)) min(points(:,2)) 0]);
%XYZ = bsxfun(@plus,XYZ,[1 1 0]);

X = uint32(scale*XYZ(:,1))+1;
Y = uint32(scale*XYZ(:,2))+1;

xl = [min(X) max(X)];
yl = [min(Y) max(Y)];
zl = [min(XYZ(:,3)) max(XYZ(:,3))];

%xlin = double(1:xl(2));
%ylin = double(1:yl(2));
%[Xg,Yg] = meshgrid(xlin,ylin);
%f = scatteredInterpolant(scale*XYZ(:,1),scale*XYZ(:,2),XYZ(:,3),'natural','none');
%Z = f(Xg,Yg);

ind = sub2ind([yl(2) xl(2)],Y,X);
Z = accumarray(ind,XYZ(:,3),[xl(2)*yl(2) 1],@max);
Z = reshape(Z,[yl(2) xl(2)]);

% Mask
mask = false(size(Z));
idx = unique(sub2ind(size(Z),Y,X));
mask(idx) = true;
Z(~mask) = min(Z(:));

%figure; imagesc(Z); axis image;

%k = boundary(scale*XYZ(:,1),scale*XYZ(:,2),1);
%hold on;
%plot(X(k),Y(k),'r');
%hold off;

%% Select control points
Zs = flipud(Z);
Zs = Zs - min(Zs(:));
Zs = Zs/max(Zs(:));

[scenePoints,mapPoints] = cpselect(Zs,mapImage,'Wait',true);

%% Warp map
maptform = fitgeotrans(scenePoints,mapPoints,'similarity');
Rmap = imref2d(size(mapImage));
registered = imwarp(Zs, maptform, 'OutputView', Rmap);
figure; imshowpair(mapImage,registered,'Scaling','independent');

view(2);
strPNG = [outFolder filesep dataSet '_map_fused.png'];
imwrite(imfuse(mapImage,registered,'Scaling','independent'),strPNG);

%% Point cloud with map
Rimap = imref2d(size(Zs));
registeredMap = imwarp(mapImage, maptform.invert, 'OutputView', Rimap);

%figure;
%surface(Xg,Yg,zeros(size(Xg)),flipud(registered),...
%    'FaceColor','texturemap',...
%    'EdgeColor','none',...
%    'CDataMapping','direct');
%hold on; showPointCloud([scale*XYZ(:,1) scale*XYZ(:,2) XYZ(:,3)-min(XYZ(:,3))]); hold off;

hp = figure;

rotaxis = size(registeredMap,1) > size(registeredMap,2);

if rotaxis
   imshow(zeros(size(registeredMap,2),size(registeredMap,1),3),'InitialMagnification',100);
else
   imshow(zeros(size(registeredMap,1),size(registeredMap,2),3),'InitialMagnification',100);
end

clf;

if ismatrix(registeredMap)
   registeredMap = registeredMap(:,:,[1 1 1]);
end

showPointCloud([scale*XYZ(:,1) scale*XYZ(:,2) XYZ(:,3)-min(XYZ(:,3))], RGB);
hold on; imagesc(flipud(registeredMap)); axis off; hold off;

view(2);

if rotaxis
   view(-90,90)
end

% strPNG = [outFolder filesep dataSet '_map_pointcloud.png'];
% expandAxes(hp);
% set(gca,'Position',[0 0 1 1]);
% savePNG(hp,strPNG);

%% Save mat file
save(outFile,'Zs','scenePoints','mapPoints','maptform','-append');
