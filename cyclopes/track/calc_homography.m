function [H_new, WarpedImage, norm_x, CurrentImage] = calc_homography(tracking_param,ReferenceImage,H)    
    % Iterative non-linear homography estimation
    [H_new, WarpedImage, norm_x] = TrackImageSL3(ReferenceImage, CurrentImage, H, tracking_param);
end