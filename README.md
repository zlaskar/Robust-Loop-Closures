# Robust-Loop-Closures

The Robust-Loop-Closures provides a system to perform robust loop closure using evidences from visual landmarks and initial raw odometry (which has drift) to perform drift correction. The key concept is the energy function for global optimization which can faithfully reject wrong visual correspondences (arising from repetitive scenes like pictures, doors commonly found in an indoor environment) and simultaneously perform globally consistent loop closures. For more information refer to our ICIP'16 paper : 

Z. Laskar, S. Huttunen, D. Herrera C., E. Rahtu and J. Kannala, "Robust loop closures for scene reconstruction by combining odometry and visual correspondences," 2016 IEEE International Conference on Image Processing (ICIP), Phoenix, AZ, USA, 2016, pp. 2603-2607.

If you use our paper or codes for any of your work, please cite the above paper.

The implementation uses openMVG and associated third party libraries (e.g. ceres). 

Link to video : https://www.youtube.com/watch?v=peXrP374MVI
