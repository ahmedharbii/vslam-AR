%% warp 2d image using polygon points
function warped_2d_image = warp_2d_image(image_to_warp, background_image, polygon)
    matchedPtsDistorted = [[0,0];[size(image_to_warp,1),0];...
        [size(image_to_warp,1),size(image_to_warp,2)];...
        [0,size(image_to_warp,2)]];
    
    matchedPtsOriginal = polygon(1:2,1:end-1)';
    
    [tform,inlierIdx] = estimateGeometricTransform2D(matchedPtsDistorted,matchedPtsOriginal,'projective');
    
    outputView = imref2d(size(background_image.I));
    warped_2d_image = imwarp(image_to_warp,tform,'OutputView',outputView);
return;