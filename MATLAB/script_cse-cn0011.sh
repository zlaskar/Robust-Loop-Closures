#!/bin/bash

export LD_LIBRARY_PATH=/lib64:${LD_LIBRARY_PATH}

projectPath=$1
inputImagePath=${projectPath}
binPath=/research/imag/external/SfM/openMVG/install-0.9/bin

export PATH=${binPath}:${PATH}

mkdir ${inputImagePath}"/SfM"

outputMatchesPath=${inputImagePath}"/SfM/matches"
outputReconstructionPath=${inputImagePath}"/SfM/reconstruction"

mkdir ${outputReconstructionPath}

# 0 - Set number of processing threads
export OMP_NUM_THREADS=32

# 1 - Image listing
# ( list image information - size + focal length )

openMVG_main_SfMInit_ImageListing -i ${inputImagePath} -o ${outputMatchesPath} -d /research/imag/development/GoogleTango/data/openmvg/cameraGenerated.txt -k "1040.47;0;634.037;0;1040.63;365.999;0;0;1" -g 1

# 2 - Features computation
# ( Compute per image a list of features & descriptors )

openMVG_main_ComputeFeatures -i ${outputMatchesPath}/sfm_data.json -o ${outputMatchesPath} -p HIGH

# 3 - Matches computation
# ( Compute reliable matches between the picture set )

openMVG_main_ComputeMatches -i ${outputMatchesPath}/sfm_data.json -o ${outputMatchesPath} -g e -r 0.8

# 4 - Global reconstruction
export OMP_NUM_THREADS=1
openMVG_main_GlobalSfM -i ${outputMatchesPath}/sfm_data.json -m ${outputMatchesPath} -o ${outputReconstructionPath}/global

# View the SfM statistic report
# firefox ${outputReconstructionPath}/global/SfMReconstruction_Report.html

openMVG_main_ComputeSfM_DataColor -i ${outputReconstructionPath}/global/sfm_data.json -o ${outputReconstructionPath}/global/colorized.ply

# View the SfM result (camera poses and structure point cloud)
# meshlab "${outputReconstructionPath}/global/colorized.ply"

# pcmanfm ${outputReconstructionPath}/global
