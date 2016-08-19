

function depthTangoProj = getTangoProjDepth(xImg, yImg, TangoProjImg)

        depthMat = TangoProjImg;
        depthTangoProj = getAvgKNNDepth(depthMat, xImg, yImg); 
        

end



function avgDepth = getAvgKNNDepth(matFile, x, y)

%%Get average depth from 3*3 or 5*5 neighbour
avgDepth = 0;
nonZeroDepth = 0;
kNN = 5;
k = floor(kNN/2);
for iy = y-k:y+k
    for ix = x-k:x+k
        if ix > 1 && ix < 1280 && iy > 1 && iy < 720
            if abs(matFile(ix, iy)) > 0
                nonZeroDepth = nonZeroDepth + 1;
            end
            avgDepth = avgDepth + matFile(ix, iy);
        end
    end
end
if nonZeroDepth == 0
    avgDepth = 0;
else
    avgDepth = avgDepth/nonZeroDepth;
end

end