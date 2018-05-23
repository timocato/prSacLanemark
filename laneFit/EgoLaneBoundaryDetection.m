%///////////////////////////////
%///// By Yingmao Li  @ UTD ////
%///// All Rights Reserved  ////
%///////////////////////////////

classdef EgoLaneBoundaryDetection
    properties
        winLen; % - window length for computing road mark confidence
        predictRadius;
    end
    methods

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Recursive Road Mark Fitting %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [params, inliers_l, inliers_r, predMap, pl, pr] ...
                = recursiveRoadMarkDetection(~, measurement,initAnchor, ...
                ridge_map, image_width, win_len, win_width, n_iter, ...
                inlier_thr)
            isReachLimit = 0;
            isFalsePos = 0;
            isNoData2 = 0;
            predMap = [];
            measurementA = measurement;
            ridgeMap = ridge_map;


            % - set up init measurment in the close range
            x = measurementA(2, :);
            y = measurementA(1, :);
            % - close range road mark fitting
            id_closeRange = y <= initAnchor; % 1200
            y(id_closeRange) = [];
            x(id_closeRange) = [];
            id_left = ~(x > image_width/2 & y > initAnchor);
            id_right = ~(x <= image_width/2 & y > initAnchor);
            x_left = x(id_left);
            x_right = x(id_right);
            y_left = y(id_left);
            y_right = y(id_right);
            meas_l = [x_left; y_left];
            meas_r = [x_right; y_right];
            
            if(length(meas_l) < 5 || length(meas_r) < 5)
                measurementA = measurement;
                ridgeMap = ridge_map;
                x = measurementA(2, :);
                y = measurementA(1, :);
                % - close range road mark fitting
                id_closeRange = y <= initAnchor; % 1200
                y(id_closeRange) = [];
                x(id_closeRange) = [];
                id_left = ~(x > image_width/2 & y > initAnchor);
                id_right = ~(x <= image_width/2 & y > initAnchor);
                x_left = x(id_left);
                x_right = x(id_right);
                y_left = y(id_left);
                y_right = y(id_right);
                meas_l = [x_left; y_left];
                meas_r = [x_right; y_right];
            end
            
            anchor = initAnchor;
            while(isReachLimit == 0 && isNoData2 == 0 && isFalsePos == 0)
                [params, inliers_l, inliers_r] = fitEgoLaneMarker(meas_l, meas_r, ...
                    n_iter, inlier_thr);
                [mask, data_l, data_r] = searchForLaneMarking(params, anchor, ...
                    win_len, win_width, ridgeMap);
                meas_l = [inliers_l, data_l];
                meas_r = [inliers_r, data_r];
                anchor = anchor - win_len;
                % - stop criteria 3.1
                if anchor <= -win_len  
                    isReachLimit = 1;
                end
                % - stop criteria 3.2
                if(~isempty(mask))
                    maskDist = abs(mask(2, 1) - mask(2, 5));
                else
                    maskDist = 10000;
                end
                thrUpperLim = 600;
                thrLowerLim = 70;
                if (maskDist > thrUpperLim || maskDist < thrLowerLim)
                    isFalsePos = 1;
                end
                % - stop criteria 3.3
                if (isempty(data_l) && isempty(data_r))
                    isNoData2 = 1;
                elseif (isempty(data_l) || isempty(data_r))

                end
            end
            
            range = max(min(meas_l(2, :)), min(meas_r(2, :)));
            y = range:100:1900;
            xl = ppval(params(1), y);
            xr = ppval(params(2), y);
            pl = [xl; y];
            pr = [xr; y];
            
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% System Predict (KF/EKF)%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [stateEst, covPredictEst] = sysPredict(~, curState, ...
                covPredictPre, QMat, deltaT)
            xL = curState(1);
            veloZ = curState(2);
            accZ = curState(3);
            yawZ = curState(4);
            gyroZ = curState(5);

            Ft = setDynamicMat(yawZ, veloZ, accZ, deltaT);
            xLt = xL + veloZ * tan(yawZ) * deltaT + (1/2) * accZ * tan(yawZ) * deltaT^2;
            veloZt = veloZ + accZ * deltaT;
            yawZt = yawZ + gyroZ * deltaT;
            stateEst = [xLt, veloZt, accZ, yawZt, gyroZ]';
            covPredictEst = Ft * covPredictPre * Ft' + QMat;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% System Update (KF/EKF)%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [statePredict, covPredict] = sysUpdate(~, stateEst, ...
                covPredictEst, RMat, params, imWidth, zeroDist, gyro, acc, isIMU)
            H = setMeasurementMat(isIMU);
            zT = measurementEgoLane(params, imWidth, zeroDist, isIMU, gyro, acc);
            res = zT - H * stateEst;
            resCov = H * covPredictEst *H' + RMat;
            gainSubOpt = covPredictEst * H' /resCov;
            statePredict = stateEst + gainSubOpt * res;
            covPredict = (eye(length(stateEst)) - gainSubOpt * H) * covPredictEst;
        end


        
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% System Dynamics Matrix (EKF)%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dynamicsMat = setDynamicMat(yawZ, veloZ, accZ, deltaT)
    dynamicsMat = [1, tan(yawZ) * deltaT, 0.5 * tan(yawZ) * deltaT^2, ...
        veloZ * sec(yawZ)^2 * deltaT + 0.5 * accZ * sec(yawZ)^2 * deltaT^2, 0;
        0, 1, deltaT, 0, 0;
        0, 0,      1, 0, 0;
        0, 0,      0, 1, deltaT;
        0, 0,      0, 0, 1];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Ego Lane Marker Fitting %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% - input : measurements: 2 x n array, image_with, n_iter(number of
% - maxmum iterations, pred_params(predicted parameters)
function [params, inliers_left, inliers_right] = ...
    fitEgoLaneMarker(meas_l, meas_r, n_iter, inlier_thr)

    x_left = meas_l(1, :);
    x_right = meas_r(1, :);
    y_left = meas_l(2, :);
    y_right = meas_r(2, :);


    best_inlier_left = 0;
    best_inlier_right = 0;

    best_hypo_left = [];
    best_hypo_right = []; 
    inliers_left = [];
    inliers_right = [];

    % - MODEL FIT for left and right
    % - for this many iterations 

    % - Initialization Ego Lane In the close range : START
    for i = 1 : n_iter
        % - introduce previous polynomial
        % - sample data uniformly, get 5 samples
        if(length(y_left) > 5 && length(y_right) > 5)
            % - sampling simutanousely for both left and right lane markers
            [y_leftSample, id_leftSample] = datasample(y_left, 5);
            [y_rightSample, id_rightSample] = datasample(y_right, 5);
            x_leftSample = x_left(id_leftSample);
            x_rightSample = x_right(id_rightSample);

            % - fit two splines
            breaksLeft = [min(y_left) + (max(y_left) - min(y_left))/3, ...
                min(y_left) + (max(y_left) - min(y_left))*2/3]; 
            breaksRight = [min(y_right) + (max(y_right) - min(y_right))/3, ...
                min(y_right) + (max(y_right) - min(y_right))*2/3];
            poly_left = spline_PRSAC(y_leftSample, x_leftSample, breaksLeft, 3, 0.1);
            poly_right = spline_PRSAC(y_rightSample, x_rightSample, breaksRight, 3, 0.1);

            % - test splines
            x_hat_left = ppval(poly_left,y_left);
            x_hat_right = ppval(poly_right, y_right);
            error_left = (x_hat_left - x_left).^2;
            error_right = (x_hat_right - x_right).^2;

            % - inlier id
            in_id_left = sqrt(error_left) <= inlier_thr;
            in_id_right = sqrt(error_right) <= inlier_thr;
            n_inlier_left = sum(in_id_left);
            n_inlier_right = sum(in_id_right);

            % - keep the best hypo only
            if (n_inlier_left > best_inlier_left)
                best_hypo_left = poly_left;
                best_inlier_left = n_inlier_left;
                inliers_left = [x_left(in_id_left); y_left(in_id_left)];
            end

            if (n_inlier_right > best_inlier_right)
                best_hypo_right = poly_right;
                best_inlier_right = n_inlier_right;
                inliers_right = [x_right(in_id_right); y_right(in_id_right)];
            end
        end
    end
    % - Initialization Ego Lane In the close range : END
    params = [best_hypo_left; best_hypo_right];
end


function [mask, data_l, data_r] = searchForLaneMarking( ...
        params, anchor, w_len, w_width, ridge_map)
    mask = [];
    data_l = [];
    data_r = [];

    if(~isempty(params))
        params_left = params(1);
        params_right = params(2);
        % - compute tangent of the Ego Lane Boundary
        tan_y_left = ppval(ppdiff(params_left), anchor);
        tan_y_right = ppval(ppdiff(params_right), anchor);
        % - generate ROI polygon
        p1x_l_r = anchor;
        p1y_l = ppval(params_left, anchor) + w_width/2;
        p1y_r = ppval(params_right, anchor) + w_width/2;
        p2x_l_r = anchor;
        p2y_l = ppval(params_left, anchor) - w_width/2;
        p2y_r = ppval(params_right, anchor) - w_width/2;
        p3x_l_r = anchor - w_len;
        p3y_l = ppval(params_left, anchor) - w_len * tan_y_left - w_width/2;
        p3y_r = ppval(params_right, anchor) - w_len * tan_y_right - w_width/2;
        p4x_l_r = anchor - w_len;
        p4y_l = ppval(params_left, anchor) - w_len * tan_y_left + w_width/2;
        p4y_r = ppval(params_right, anchor) - w_len * tan_y_right + w_width/2;
        mx = [p1x_l_r, p2x_l_r, p3x_l_r, p4x_l_r, p1x_l_r, p2x_l_r, p3x_l_r, p4x_l_r];
        my = [p1y_l  , p2y_l  , p3y_l  , p4y_l  , p1y_r  , p2y_r  , p3y_r  , p4y_r  ];

        mask_l_x = mx(1: 4);
        mask_r_x = mx(5: end);
        mask_l_y = my(1: 4);
        mask_r_y = my(5: end);

        BW_l = roipoly(ridge_map, mask_l_y, mask_l_x);
        BW_r = roipoly(ridge_map, mask_r_y, mask_r_x);

        [meas_l_col, meas_l_row] = find(BW_l & ridge_map);
        [meas_r_col, meas_r_row] = find(BW_r & ridge_map);
        data_l = [meas_l_row'; meas_l_col'];
        data_r = [meas_r_row'; meas_r_col'];
        mask = [mx; my];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Measurements Matrix (KF/EKF)%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function measMat = setMeasurementMat(isIMU)
    if(isIMU == 1)
        measMat = [1 0 0 0 0;
                   0 0 1 0 0;
                   0 0 0 1 0;
                   0 0 0 0 1];
    else
        measMat = [1 0 0 0 0;
                   0 0 0 1 0];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Measurements for Tracker %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% - get measurements for tracker (PRSAC update)
% - all params/measurement here are in (pixels and degrees)
function measTracker = measurementEgoLane(params, imWidth, ...
        zeroDist, isIMU, gyro, acc)
    vLoc = imWidth/2; % - get the cam location 
    paramL = params(1); % - get the left elb params
    xL =  vLoc - ppval(paramL, zeroDist);
    tanL = ppval(ppdiff(paramL), zeroDist);
    if(~isIMU == 1)
        measTracker = [xL, atan(tanL)]';
    else
        measTracker = [xL, acc, atan(tanL), gyro]';
    end
end