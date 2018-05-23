# Predictive RANSAC with application on Ego Lane Mark Fit and Tracking
Ego lane mark fitting from a front mount camera using Predictive RANSAC
By Yingmao, Li and Nicholas Gans
Citing
If you enjoy the code, please cite these papers

@InProceedings{YL2014,
  Title                    = {Multiple lane boundary detection using a combination of low-level image features},
  Author                   = {Yingmao Li and Iqbal, A. and Gans, N.R.},
  Booktitle                = {Intelligent Transportation Systems, IEEE International Conference on},
  Year                     = {2014},
  Month                    = {Oct},
  Pages                    = {1682-1687}
}

@Article{YL2017,
  author   = {Yingmao Li and Nicholas R. Gans},
  title    = {Predictive RANSAC: Effective model fitting and tracking approach under heavy noise and outliers},
  year     = {2017},
  volume   = {161},
  pages    = {99 - 113},
  issn     = {1077-3142},
  doi      = {https://doi.org/10.1016/j.cviu.2017.05.013},
  url      = {http://www.sciencedirect.com/science/article/pii/S107731421730108X},
  journal  = {Computer Vision and Image Understanding},
  keywords = {Robust estimation, Outlier removal, RANSAC, Kalman filtering},
}


To use: 
You will need Matlab, image processing toolbox, autonomous driving toolbox. 
The curve fitting module is borrowed from https://www.mathworks.com/matlabcentral/fileexchange/13812-splinefit

To run the code: 
1. download and unzip Kitti road dataset from http://www.cvlibs.net/datasets/kitti/raw_data.php?type=road, and use the synced+rectified data. 
2. Open up the main.m file, modify the imgPath and point to the dataset image folder. 
3. Press "run" or hit F5 to check the result. 


Thanks you
Yingmao, Li
