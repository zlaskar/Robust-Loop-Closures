function [ points, timestamp ] = vtkImportPoints( fileName )
%VTKPOINTS Summary of this function goes here
%   Detailed explanation goes here

%%
% DataOutputStream out = new DataOutputStream(new BufferedOutputStream(
%                     new FileOutputStream(file))); 
% 
fid = fopen(fileName,'r');

%% Header 
% out.write(("# vtk DataFile Version 3.0\n" +
%                     "vtk output\n" +
%                     "BINARY\n" +
%                     "DATASET POLYDATA\n" +
%                     "POINTS " + xyzIj.xyzCount + " float\n").getBytes());
%
headerLength = 5;

for i=1:headerLength
   line = fgets(fid);
end

pointCount = sscanf(line, '%*s %d %*s');

%% Pointcloud
% for (int i = 0; i < xyzIj.xyzCount; i++) {
%    out.writeFloat(myBuffer.getFloat(3 * i * 4));
%    out.writeFloat(myBuffer.getFloat((3 * i + 1) * 4));
%    out.writeFloat(myBuffer.getFloat((3 * i + 2) * 4));
% }

points = fread(fid,[3,pointCount],'float=>float',0,'l');
points = points';

%% Vertices (not needed actually)
% out.write(("\nVERTICES 1 " + String.valueOf(xyzIj.xyzCount + 1) + "\n").getBytes());
% out.writeInt(xyzIj.xyzCount);
% for (int i = 0; i < xyzIj.xyzCount; i++) {
%    out.writeInt(i);
% }

% [~] = fgets(fid);
% line = fgets(fid);
% verticeCount = sscanf(line, '%*s %*d %d');
% [~] = fread(fid,verticeCount,'int=>int',0,'l');

%% Timestamp
% out.write(("\nFIELD FieldData 1\n" + "timestamp 1 1 float\n").getBytes());
% out.writeFloat((float) xyzIj.timestamp);

[~] = fgets(fid);
[~] = fgets(fid);
line = fgets(fid);
timestamp = fread(fid,1,'float=>float',0,'l');

fclose(fid);

if isempty(timestamp)
   timestamp = NaN;
   %movefile(fileName,[fileName '.invalid']);
   disp(['Corrupted file: ' fileName]);
end

end

