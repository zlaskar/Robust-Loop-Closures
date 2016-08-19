function [ posesLearning,posesService ] = showTrajectories( tangoPose )
%SHOWTRAJECTORIES Summary of this function goes here
%   Detailed explanation goes here

figure;

idx = 1:size(tangoPose,1);

posesLearning = tangoPose(:,5)>0;
posesService = tangoPose(:,5)<0;

timediff = tangoPose(2:end,5) - tangoPose(1:end-1,5);
validPoses = true(size(tangoPose,1),1);
validPoses(2:end) = timediff<0; % Mask out point cloud poses

posesService = validPoses & posesService;
posesLearning = ~validPoses & posesLearning;

tangoPose(posesService,6:8) = bsxfun(@minus,tangoPose(posesService,6:8),tangoPose(find(posesService,1,'first'),6:8));
tangoPose(posesLearning,6:8) = bsxfun(@minus,tangoPose(posesLearning,6:8),tangoPose(find(posesLearning,1,'first'),6:8));

x = double(tangoPose(:,6));
y = double(tangoPose(:,7));
z = double(tangoPose(:,8));

xs = x(posesService);
ys = y(posesService);
zs = z(posesService);

xl = x(posesLearning);
yl = y(posesLearning);
zl = z(posesLearning);

h = animatedline(0,0,0,'Color','r');
clearpoints(h);

h2 = animatedline(0,0,0,'Color','b');
clearpoints(h2);

view(3);
view(0,90);

%for i = 1:size(tangoPose,1)
for i = 1:min(sum(posesService),sum(posesLearning))
   
   %if ismember(i,idx(posesLearning))
      addpoints(h,xs(i),ys(i),zs(i));
   %elseif ismember(i,idx(posesService))
      addpoints(h2,xl(i),yl(i),zl(i));
    %end
    drawnow;
end

end

