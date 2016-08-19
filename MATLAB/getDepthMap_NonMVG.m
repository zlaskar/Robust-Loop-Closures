%% function to get depth map for an Image from point cloud
function [dMap,RGBStruct, tmp] = getDepthMap_NonMVG(ptCld, img)
% % % , k11, k21, k31,  w1, h1)

        %% temp
        camparams = [1040.47; 1040.63; 634.03700000000003; 365.99900000000002; 0; 0; 0]; %White
        
        % w = 640; h = 360;
        w1 = 1280; h1 = 720;
        % Focal length
        fx1 = camparams(1);
        fy1 = camparams(2);
        
        % Principal point
        u01 = camparams(3);
        v01 = camparams(4);
        
        % Distortion parameters
        k11 = camparams(5);
        k21 = camparams(6);
        k31 = camparams(7);
        
        K1 = [fx1  0 u01;
            0 fy1 v01;
            0  0  1];

        % Normalized image coordinates
        xn = double(ptCld(:,1)./ptCld(:,3));
        yn = double(ptCld(:,2)./ptCld(:,3));
        % Distance
        r2 = xn.^2+yn.^2;
        
        % Distorted coordinates
        xc = xn.*(ones(length(xn),1) + k11 * r2 + k21 * r2.^2 + k31 * r2.^3);
        yc = yn.*(ones(length(yn),1) + k11 * r2 + k21 * r2.^2 + k31 * r2.^3);
        % Distorted image pixel coordinates
        pp = K1*[xc'; yc'; ones(1,length(xc))];
        xp = pp(1,:)';
        
        yp = pp(2,:)';
        
        %%%Ortho projection
        
        R1 = [1 0 0;0 1 0;0 0 1];
        C1 = [0 0 0];
        
        dMap = double(zeros([w1,h1]));
        RGBStruct = zeros(size(ptCld,1),3);
        tmp = zeros(size(ptCld,1),1);
        
        for imgCor = 1 : size(ptCld,1)
            xCor = round(xp(imgCor,1));
            yCor = round(yp(imgCor,1));
            if xCor > 0 && yCor > 0 && xCor < w1 && yCor < h1
                
                %TangoPoint variable to store points that fall within the image
                %plane
                
                % % %                 TngPts(imgCor,:) = pts(imgCor,:);
                
%                 dCor = getOrthProj(R1, C1, ptCld(imgCor,1:3), K1);
                %                 depthMat(xCor, yCor, 1) = 1;
%                 dMap(xCor, yCor) = double(dCor);

                % Assign the index imgCor in tmp 1 to indicate the ptCloud
                % is visible
                tmp(imgCor) = 1;

                RGBStruct(imgCor,:) = img(yCor, xCor,:);
                % get the RGB Color
                
                
% % %                 %% Modification (Hererra)
% % %                 kNN = 3;
% % %                 k = floor(kNN/2);
% % %                 for iy = yCor-k:yCor+k
% % %                     for ix = xCor-k:xCor+k
% % %                         if ix > 1 && ix < 1280 && iy > 1 && iy < 720
% % %                             
% % %                             depthMat(ix, iy) = double(dCor);
% % %                         end
% % %                     end
% % %                 end
% % % % % %                 
            end
            
        end


end