function [ ] = savePNG( fig, strFile )
%SAVEPNG Summary of this function goes here
%   Detailed explanation goes here

set(fig,'PaperPositionMode','auto');
print(fig,strFile,'-dpng','-r0');

end

