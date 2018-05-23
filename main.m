%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%         By Yingmao Li         %%%
%%% University of Texas at Dallas %%%
%%%      All Rights Reserved      %%%
%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear; clc;
addpath(genpath('./splineFit'));
addpath(genpath('./laneFit'));
addpath(genpath('./ridgeDetect'));
warning('off','all');
warning;
% - modify and point them into correct directories
imgDir = 'D:\data_backup\2011_09_26_drive_0028_sync\2011_09_26\2011_09_26_drive_0028_sync';
res = 5;

% - set up parameters for the PredictiveRANSAC model fiting and tracker
prSac = PredictiveRansac;
prSac.model = 'EgoLaneMarker';
prSac.noi = 100;
prSac.thr = 3; % inlier threshold in pixel
prSac.numOfModels = 1; 
prSac.windowLen = 500;
prSac.windowWidth = 100;
prSac.imgWidth = 640;
prSac.imgHeight = 1800;
prSac.app = EgoLaneBoundaryDetection;
stateCurr = [70, 0, 0, 0, 0]';
estNoiseCov = 0.01*eye(5);
prSac.measNoiseCov = diag([0.01, 0.01]);
prSac.procNoiseCov = diag([0.1, 0.1, 0.1, 0.5, 0.7]);
prSac.app.winLen = 5; 
prSac.app.predictRadius = 60;
poly = [];



% - bird's eye view params: START
focalLength    = [721.5377, 721.5377]; % [fx, fy] in pixel units
principalPoint = [609.5593, 172.8540]; % [cx, cy] optical center in pixel coordinates
imageSize      = [375, 1242];           % [nrows, mcols]
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
height = 2.7;    % mounting height in meters from the ground
pitch  = 0;        % pitch of the camera in degrees
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);
distAheadOfSensor = 70;
spaceToOneSide = 10;
bottomOffset = 10;
outView = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide];
imageSize = [NaN, 640];
birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);
vehicleROI = outView - [-1, 2, -3, 3];
% - bird's eye view params: END

figure; 
for i = 1:1000 % - total number of images 
    cla;
    % - ground truth and image
    img = imread(sprintf('%s/image_%02d/data/%010d.png',imgDir,2,i));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Preprocess and Measure %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    img_gray = rgb2gray(img);
    img_gray_bird = adapthisteq(img_gray, 'NumTiles', [10 2], ... 
        'clipLimit', .005, 'Distribution', 'uniform');

    img_bird = transformImage(birdsEyeConfig, img_gray_bird); 
    birdsEyeViewBW = ridge2D(img_bird, 3.5, 1.5);  
    % - morphological open 
    se = strel('square', 6);
    birdsEyeViewBW = imopen(birdsEyeViewBW, se);
    birdsEyeViewBW(:, 1:30) = 0;
    birdsEyeViewBW(:, 610:640) = 0;
    % - add ROI to trim the ego lane
    birdsEyeViewBW(1200:end, 1:150) = 0;  
    birdsEyeViewBW(1200:end, 490:end) = 0; 
    [meas_col, meas_row] = find(birdsEyeViewBW); 

    
    prSac.ridgeMap = birdsEyeViewBW;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%      Model Fit and Tracking      %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % - Model fitting and tracking with predictive RANSAC
    % - We only detect and predict ego lane (2 lane markers) 
    [stateEst, estNoiseCovEst] = prSac.step(stateCurr, ...
        estNoiseCov, .5, []);  
    
    stateCurr = stateEst;
    estNoiseCov = estNoiseCovEst;  
    prSac.measurement = [meas_col'; meas_row']; % - load measurement
    [poly, inliers_r, inliers_l, predMap, p1, p2] = prSac.modelFitting();
    p1 = sensor.vehicleToImage(imageToVehicle(birdsEyeConfig, p1'));
    p2 = sensor.vehicleToImage(imageToVehicle(birdsEyeConfig, p2'));
    if(length(poly) == 2)
        [stateUpdate, updateNoiseCov] = prSac.update(stateEst,...
            estNoiseCov,poly, [], [], 0);
        stateCurr = stateUpdate;
        estNoiseCov = updateNoiseCov;
    end
    


    % - For the display purpose
    if(~isempty(inliers_l))
        elb2Image_l = vec2ridgemap(round((sensor.vehicleToImage(...
            imageToVehicle(birdsEyeConfig, inliers_l')))'), img);
    end
    if(~isempty(inliers_r))
        elb2Image_r = vec2ridgemap(round((sensor.vehicleToImage(...
            imageToVehicle(birdsEyeConfig, inliers_r')))'), img);
    end
    ch_r = img(:, :, 1);
    ch_g = img(:, :, 2);
    ch_b = img(:, :, 3);
    id_right = elb2Image_r == 255; 
    id_left = elb2Image_l == 255;
    ch_r(id_left) = 255;
    ch_g(id_left) = 0;
    ch_b(id_left) = 0;
    ch_r(id_right) = 0;
    ch_g(id_right) = 255;
    ch_b(id_right) = 0;
    img(:, :, 1) = ch_r;
    img(:, :, 2) = ch_g;
    img(:, :, 3) = ch_b;
    imshow(img);
    frame = getframe(gcf);
    
end


