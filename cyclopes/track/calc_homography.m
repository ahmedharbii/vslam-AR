function [H, WarpedImage, norm_x, CurrentImage] = calc_homography(tracking_param,capture_params,file_I,ReferenceImage,H, i)
    CurrentImage = read_current_image(file_I, capture_params);
    if(tracking_param.changereference && tracking_param.changereference_key)
        Htrack = eye(3,3);
    else
        Htrack = H(:,:,i-1); %to pass it to TrackImageSL3
    end
    
    % Iterative non-linear homography estimation
    [H(:,:,i), WarpedImage, norm_x] = TrackImageSL3(ReferenceImage, CurrentImage, Htrack, tracking_param);
end