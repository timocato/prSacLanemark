%///////////////////////////////
%///// By Yingmao Li, @ UTD ////
%///// All Rights Reserved  ////
%///////////////////////////////
%
% output: ridge map on an image
% input: image, sigma (similar to the width of the road mark
%        thr_1 : threshold on ridge intensity
%        thr_2 : threshold on second order derivative
function ridge = ridge2D(img, sigma, thr_1)
    %--compute Hessian---------
    G1=fspecial('gauss',21, sigma); % - size best 21
    [hx,hy] = gradient(G1);   
    [hxx,hxy] = gradient(hx);
    [~,hyy] = gradient(hy);  
    
    
    gx = imfilter(double(img), hx);
    gy = imfilter(double(img), hy);
    gxx = imfilter(double(img), hxx);
    gxy = imfilter(double(img), hxy);
    gyy = imfilter(double(img), hyy);
    
    %--Firt order derivative cross the ridge
    luu = gx.*gy.*(gxx - gyy) - (gx.*gx - gy.*gy).*gxy;
    
    % - eigen value of hessian
    ridge1 = (gxx + gyy + sqrt(gxx.*gxx + 4.*gxy.*gxy - 2.*gxx.*gyy + gyy.*gyy))./2;
    ridge2 = (gxx + gyy - sqrt(gxx.*gxx + 4.*gxy.*gxy - 2.*gxx.*gyy + gyy.*gyy))./2;
    
    
    check_size = abs(ridge1) > abs(ridge2);
    check_sign1 = ridge1 > 0;
    check_sign2 = ridge2 > 0;
    
    ridge1(~check_size) = 0;
    ridge2(check_size) = 0;
    ridge1(check_sign1) = 0;
    ridge2(check_sign2) = 0;
    
    % - ridge intensity
    ridge = abs(ridge1+ridge2);
    % - get ridge direction 
    ridge_angle = acos(sqrt((1 + (gxx - gyy)./sqrt((gxx - gyy).^2 + 4*gxy.*gxy))./2));
    
    % - suppress ridge that has a intensity less than given value

    ridge(ridge < thr_1) = 0;     
    ridge(abs(luu) >= 100) = 0;
    ridge(ridge ~= 0) = 255;
    ridge(abs(ridge_angle)/pi < 0.4) = 0; % set limit on the ridge direction
end