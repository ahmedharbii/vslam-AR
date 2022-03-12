function [ReferenceImage, ReferenceImage_right, H_left, H_right, H_left_right,...
    norm_x_left, norm_x_right, norm_x_left_right]...
    = track_stereo(capture_params,image_num_string, tracking_param, i,...
    ReferenceImage, ReferenceImage_right, H_left, H_right, H_left_right,norm_x_left,norm_x_right, ...
    initializer, use_optimized)

%% Calculate Left Homography
% read the image
capture_params.data_dir = [pwd '\Versailles_canyon\Left\'];
CurrentImage = read_current_image(capture_params, image_num_string);
tracking_param.changereference = change_ref_or_not(norm_x_left, tracking_param);
% track
[ReferenceImage, H_left, WarpedImage, norm_x_left] =...
    track(tracking_param,ReferenceImage,CurrentImage,H_left,i);

%% Calculate Right Homography
capture_params.data_dir = [pwd '\Versailles_canyon\Right\'];
CurrentImage_right = read_current_image(capture_params, image_num_string);
tracking_param.changereference = change_ref_or_not(norm_x_right, tracking_param);
[ReferenceImage_right, H_right, WarpedImage_right, norm_x_right] =...
    track(tracking_param,ReferenceImage_right,CurrentImage_right,H_right,i);

%% Calculate left-Right Homography
CurrentImage_left_right = CurrentImage_right;
[H_left_right, WarpedImage_left_right, norm_x_left_right] = ...
    track_left_right(tracking_param, ReferenceImage, CurrentImage_left_right,...
    H_left, H_right, H_left_right, i, initializer, use_optimized);

%% Display
figure(1); hold on;
DrawImagePoly('Warped Current Image Left to Left', 1, CurrentImage.I, WarpedImage.polygon);
DrawImagePoly('Warped Current Image Right to Right', 2, CurrentImage_right.I, WarpedImage_right.polygon);
DrawImagePoly('Warped Current Image Left to Right', 3, CurrentImage_left_right.I, WarpedImage_left_right.polygon);

end

