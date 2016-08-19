function ThreeD_Pts_Global = joinClustersVer2(ThreeD_ImgStruct_Global,total3DLocalToClusIncrem, imgFiles)
%% Function to bring openMVG camera poses to Tango reference frame 
%  and to bring different clusters together
%  Author : Zakaria Laskar
%  Date : 24/07/2015


% Input : output of function ChainTangoTxv2(...)

% Output : ThreeD_Pts_Global

%% Bring the MVG pose to Tango World Co-ordinate System

camparams = [1040.47; 1040.63; 634.03700000000003; 365.99900000000002; 0; 0; 0]; %White

% Focal length
fx = camparams(1);
fy = camparams(2);

% Principal point
u0 = camparams(3);
v0 = camparams(4);

% Distortion parameters
k1 = camparams(5);
k2 = camparams(6);
k3 = camparams(7);

K = [fx  0 u0;
    0 fy v0;
    0  0  1];

NumImages = size(imgFiles,1);
% NumOfClus = size(clusInd,2);
ThreeDPointCounter = zeros(1,(total3DLocalToClusIncrem));
ImagesPer3DCounter = zeros(1,(total3DLocalToClusIncrem));

for i = 1 : NumImages %imgClus
                        
            imgNum = i;%clusInd{clusNum}(i);
 
            %% Find the 3D point in Tango World Co-ordinates
         
            if size(ThreeD_ImgStruct_Global.view{imgNum},2) > 0
                
                for k = 1 : size(ThreeD_ImgStruct_Global.view{imgNum}.index_3D,2)
                    % % %                 disp (k)
                    
                    % Add the current image to the k th 3D point struct
                    ImagesPer3DCounter(ThreeD_ImgStruct_Global.view{imgNum}.index_3D(k)) = ImagesPer3DCounter(ThreeD_ImgStruct_Global.view{imgNum}.index_3D(k)) + 1; % Increment the counter storing the image num viewing the current 3D point
                    ThreeD_Pts_Global.pts{ThreeD_ImgStruct_Global.view{imgNum}.index_3D(k)}.views(ImagesPer3DCounter(ThreeD_ImgStruct_Global.view{imgNum}.index_3D(k))) = imgNum; % Store the image 
                    % Check if the 3D point is already set
                    if ThreeDPointCounter(ThreeD_ImgStruct_Global.view{imgNum}.index_3D(k)) == 0
%                        
                        ThreeDPointCounter(ThreeD_ImgStruct_Global.view{imgNum}.index_3D(k)) = 1;
                    end
                end
            end
            
            %% If not a validFile no Point cloud available
%            
            
end
%     end


end