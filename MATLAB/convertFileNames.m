function [ inputFiles ] = convertFileNames( inputFiles )
%CONVERTFILENAMES Summary of this function goes here
%   Detailed explanation goes here

if ispc
   inputFiles = strrep(inputFiles,'/research/imag/','\\samba.ee.oulu.fi\r-imag\');
   inputFiles = strrep(inputFiles,'/','\');
elseif isunix
   inputFiles = strrep(inputFiles,'\\samba.ee.oulu.fi\r-imag\','/research/imag/');
   inputFiles = strrep(inputFiles,'\','/');
end

end

