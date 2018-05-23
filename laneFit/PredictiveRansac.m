%///////////////////////////////
%///// By Yingmao Li  @ UTD ////
%///// All Rights Reserved  ////
%///////////////////////////////
%
% - Predictive RANSAC: refer to publication [1 - 3]
classdef PredictiveRansac
    properties
        noi; % - number of iteration
        thr; % - inlier threshold 
        measurement; % - measurement 
        numOfModels; % - the total number of models we are looking for
        model; % - model we want to use: 'EgoLaneMarker'
        ridgeMap; % - ridge map for selecting data easily from a sliding window
        windowLen; % - sliding window length
        windowWidth; % - sliding window Width
        imgWidth;
        imgHeight;
        % - tracker params 
        measNoiseCov;
        procNoiseCov;        
        app;
    end
    
    methods
        % - preSac model fitting stage
        function [params, inlier_l, inlier_r, predMap, pl, pr] = ...
                modelFitting(obj)
            max_iter = obj.noi; 
            inlier_thr = obj.thr;
            ridge_map = obj.ridgeMap;
            params = [];
            inlier_l = [];
            inlier_r = [];
 
            if(strcmp(obj.model, 'EgoLaneMarker'))
                [params, inlier_l, inlier_r, predMap, pl, pr] = ...
                    obj.app.recursiveRoadMarkDetection(obj.measurement,...
                    1200, ridge_map, obj.imgWidth, obj.windowLen, ...
                    obj.windowWidth, max_iter, inlier_thr);
            end
        end
        
        % - preSac model prediction stage
        % - sensorInput: can be IMU, GYRO, and other sensors. 
        function [stateEst, estNoiseCov] = step(obj, ...
                stateCurr, noiseCovCurr, deltaT, sensorInput)
            stateEst = [];
            estNoiseCov = [];
            if(strcmp(obj.model, 'EgoLaneMarker'))
                if(~isempty(sensorInput))
                    % - TODO: if we have sensor input, there will be an another
                    % story
                else
                    
                    [stateEst, estNoiseCov] = obj.app.sysPredict(stateCurr, ...
                        noiseCovCurr, obj.procNoiseCov, deltaT); 
                end
            end
        end
        % - preSac model update
        function [stateUpdate, updateNoiseCov] = update(obj, stateEst, ...
                estNoiseCov, measParams, gyro, acc, isIMU)
            stateUpdate = []; 
            updateNoiseCov = [];
            if(strcmp(obj.model, 'EgoLaneMarker'))
                [stateUpdate, updateNoiseCov] = obj.app.sysUpdate(stateEst, ...
                    estNoiseCov, obj.measNoiseCov, measParams, ...
                    obj.imgWidth, 1800, gyro, acc, isIMU);
            end
        end 
    end
end