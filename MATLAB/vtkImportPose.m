function [ tangoPoseData, Cam2Dev ] = vtkImportPose( fileName )
%VTKIMPORTPOSE Summary of this function goes here
%   Detailed explanation goes here
%%
% DataOutputStream out = new DataOutputStream(new BufferedOutputStream(
% new FileOutputStream(file)));
% 
fid = fopen(fileName,'r');

%% Header
% out.write(("# vtk DataFile Version 3.0\n" +
%            "vtk output\n" +
%            "BINARY\n" +
%            "DATASET POLYDATA\n" +
%            "POINTS " + numPoints + " float\n").getBytes());

headerLength = 5;

for i=1:headerLength
   line = fgets(fid);
end

pointCount = sscanf(line, '%*s %d %*s');

%% Points
% 
% for (int i = 0; i < numPoints; i++) {
%    out.writeFloat(mPosePositionBuffer.get(i)[0]);
%    out.writeFloat(mPosePositionBuffer.get(i)[1]);
%    out.writeFloat(mPosePositionBuffer.get(i)[2]);
% }

points = fread(fid,[3,pointCount],'float=>float',0,'b');
points = points';

%% Lines (not needed actually)
%    
% out.write(("\nLINES 1 " + String.valueOf(numPoints + 1) + "\n").getBytes());
% out.writeInt(numPoints);
% 
% for (int i = 0; i < numPoints; i++) {
%    out.writeInt(i);
% }

fgets(fid);
line = fgets(fid);
lineCount = sscanf(line, '%*s %*d %d');
[~] = fread(fid,lineCount,'int=>int',0,'b');

%% Cam2Dev Transform
%       
% out.write(("\nFIELD FieldData 1\n" +
%            "Cam2Dev_transform 16 1 float\n").getBytes());
% 
% for (int i = 0; i < cam2dev_Transform.length; i++) {
%    out.writeFloat(cam2dev_Transform[i]);
% }
% 

fgets(fid);
fgets(fid);
line = fgets(fid);
transCount = sscanf(line, '%*s %d %*d %*s');
Cam2Dev = fread(fid,transCount,'float=>float',0,'b');

%% Orientation
% out.write(("\nPOINT_DATA " + String.valueOf(numPoints) + "\n" +
%            "FIELD FieldData 2\n" +
%            "orientation 4 " + String.valueOf(numPoints) + " float\n").getBytes());
%          
% for (int i = 0; i < numPoints; i++) {
%   out.writeFloat(mPoseOrientationBuffer.get(i)[0]);
%   out.writeFloat(mPoseOrientationBuffer.get(i)[1]);
%   out.writeFloat(mPoseOrientationBuffer.get(i)[2]);
%   out.writeFloat(mPoseOrientationBuffer.get(i)[3]);
% }

fgets(fid);
fgets(fid);
fgets(fid);
line = fgets(fid);
orienCount = sscanf(line, '%*s %*d %d %*s');
orientation = fread(fid,[4,orienCount],'float=>float',0,'b');
orientation = orientation';

%% Timestamp
%             
% out.write(("\ntimestamp 1 " + String.valueOf(numPoints) + " float\n").getBytes());
% for (int i = 0; i < numPoints; i++) {
%    out.writeFloat(mPoseTimestampBuffer.get(i));
% }

fgets(fid);
line = fgets(fid);
timeCount = sscanf(line, '%*s %*d %d %*s');
timestamp = fread(fid,timeCount,'float=>float',0,'b');

fclose(fid);

%% Create final data
tangoPoseData = [orientation timestamp points];

end

