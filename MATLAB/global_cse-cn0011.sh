#!/bin/bash

export LD_LIBRARY_PATH=/lib64:${LD_LIBRARY_PATH}

projectPath=$1
cluster=$2

inputImagePath=${projectPath}

binPath=/research/imag/external/SfM/openMVG/install-0.9/bin
export PATH=${binPath}:${PATH}

outputMatchesPath=${inputImagePath}"/SfM/matches"
outputReconstructionPath="${inputImagePath}/SfM/reconstruction/clusters/${cluster}"

mkdir ${outputReconstructionPath}

# 4 - Global reconstruction
export OMP_NUM_THREADS=1
openMVG_main_GlobalSfM -i "${outputMatchesPath}/sfm_data.json" -m ${outputMatchesPath} -o ${outputReconstructionPath}

openMVG_main_ComputeSfM_DataColor -i ${outputReconstructionPath}/sfm_data.json -o ${outputReconstructionPath}/colorized.ply