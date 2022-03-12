function [H_left_right, WarpedImage_left_right, norm_x_left_right] = track_left_right(tracking_param, ReferenceImage, CurrentImage_left_right, H_left, H_right, H_left_right, i, initializer, use_optimized)
    
%% Initialize H to be used in the tracking function
if(tracking_param.changereference && tracking_param.changereference_key)
    switch initializer
        case "identity"
            Htrack = eye(3,3);
        case "hand_crafted"
            Htrack = eye(3,3);
            Htrack(:,end) = [-50;0;1];
        case "estimate"
            Htrack = inv(H_left(:,:,i)) * H_left_right(:,:,i-1) * H_right(:,:,i);
        case "prev_estimate"
            Htrack = H_left_right(:,:,i-1);
    end
else
    switch initializer
        case "estimate"
            Htrack = H_left_right(:,:,1) * H_right(:,:,i);
        case "prev_estimate"
            Htrack = H_left_right(:,:,i-1); %to pass it to TrackImageSL3
    end
end

%% Track
[H_left_right(:,:,i), WarpedImage_left_right, norm_x_left_right] = ...
    TrackImageSL3(ReferenceImage, CurrentImage_left_right, Htrack, tracking_param);

%% Update H with the new estimate
if(tracking_param.changereference && tracking_param.changereference_key)
    % This computes H2_LiRi (can be used to optimize H_LiRi)
    if use_optimized
        H1_left_right = Htrack;
        H2_left_right = H_left_right(:,:,i);
        H_left_right(:,:,i) = get_avg_h(H1_left_right, H2_left_right);
    else
        H1_left_right = H_left_right(:,:,i-1);
        H2_left_right = H_left(:,:,i) * H_left_right(:,:,i) * inv(H_right(:,:,i));
        H_left_right(:,:,i-1) = get_avg_h(H1_left_right, H2_left_right);
    end
else
    H1_left_right = H_left_right(:,:,1);
    % This computes H2_L1Ri (can be used to optimize H_L1R1)
    % diagonal H_L1Ri(since we don't change ref) * inverse of H_R1Ri 
    H2_left_right = H_left_right(:,:,i) * inv(H_right(:,:,i));

    % Update Homography between left and right cameras
    H_left_right(:,:,1) = get_avg_h(H1_left_right, H2_left_right);
end
end
