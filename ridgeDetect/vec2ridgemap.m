%///////////////////////////////
%///// By Yingmao Li, @ UTD ////
%///// All Rights Reserved  ////
%///////////////////////////////
%
% - convert ridge map to measurement vector \R^(2 x n)
% - input : ridge_vec : a 2 x n measurement vector
% -       : img_mask : an image mask to tell the dimension of the output
% - output : ridge_map : a 2d ridge map 
function ridge_map = vec2ridgemap(ridge_vec, img_mask)
    ridge_map = zeros(size(img_mask, 1), size(img_mask, 2), 'uint8'); 
    ridge_map(sub2ind(size(ridge_map),ridge_vec(2, :) ,ridge_vec(1, :))) = 255;
end