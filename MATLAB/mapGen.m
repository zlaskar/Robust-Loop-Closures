function mapGen(tx_sub,tx_sub_AL,Opt_Rot_RGB, Opt_C_RGB, t_subsampled_RGB, C_RGB, t_subsampled_RGB_AL, C_RGB_AL, RGB4Pts_subsampled, RGB4Pts_AL_subsampled)

%% plot output from our approach
fileCount = 1;
R_opt=rotationmat(Opt_Rot_RGB);
for i = 1 : 1:size(Opt_Rot_RGB,1)
if numel(tx_sub{fileCount}) == 0
% disp 'wtf'
fileCount = fileCount + 1;
openMVGPointsWorld{fileCount} = [];
continue;
end
points = double([tx_sub{fileCount} ones(size(tx_sub{fileCount},1),1)]);
pointsWorld = [R_opt{fileCount} Opt_C_RGB(fileCount,:)']*points';
pointsWorld = pointsWorld';
openMVGPointsWorld{fileCount} = pointsWorld(:,1:3);
fileCount = fileCount + 1;
end

xyz = cat(1,openMVGPointsWorld{:});
RGB = uint8(cat(1,RGB4Pts_subsampled{:}));
% % , RGB_Centers));
% 
figure('Name','MVG','Renderer','opengl'); showPointCloud(xyz, RGB);
set(gca,'visible','off');
xlabel('X');
ylabel('Y');
zlabel('Z');

%% plot output from Start of Service

fileCount = 1;
R_opt=rotationmat(t_subsampled_RGB);
for i = 1 : 1:size(t_subsampled_RGB,1)
if numel(tx_sub{fileCount}) == 0
% disp 'wtf'
fileCount = fileCount + 1;
openMVGPointsWorld{fileCount} = [];
continue;
end
points = double([tx_sub{fileCount} ones(size(tx_sub{fileCount},1),1)]);
pointsWorld = [R_opt{fileCount} C_RGB(fileCount,:)']*points';
pointsWorld = pointsWorld';
openMVGPointsWorld{fileCount} = pointsWorld(:,1:3);
fileCount = fileCount + 1;
end

xyz = cat(1,openMVGPointsWorld{:});
RGB = uint8(cat(1,RGB4Pts_subsampled{:}));
% , RGB_Centers));

figure('Name','SOS','Renderer','opengl'); showPointCloud(xyz, RGB);
set(gca,'visible','off')
xlabel('X');
ylabel('Y');
zlabel('Z');

%% plot output from Area Learning

fileCount = 1;
R_opt=rotationmat(t_subsampled_RGB_AL);
for i = 1 : 1:size(t_subsampled_RGB_AL,1)
if numel(tx_sub_AL{fileCount}) == 0
% disp 'wtf'
fileCount = fileCount + 1;
openMVGPointsWorld{fileCount} = [];
continue;
end
points = double([tx_sub_AL{fileCount} ones(size(tx_sub_AL{fileCount},1),1)]);
pointsWorld = [R_opt{fileCount} C_RGB_AL(fileCount,:)']*points';
pointsWorld = pointsWorld';
openMVGPointsWorld{fileCount} = pointsWorld(:,1:3);
fileCount = fileCount + 1;
end

xyz = cat(1,openMVGPointsWorld{:});
RGB = uint8(cat(1,RGB4Pts_AL_subsampled{:}));
% , RGB_Centers));
% 
figure('Name','AL','Renderer','opengl'); showPointCloud(xyz, RGB);
set(gca,'visible','off')
xlabel('X');
ylabel('Y');
zlabel('Z');