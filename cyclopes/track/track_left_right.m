function [H_left_right, WarpedImage_left_right, data] = track_left_right(tracking_param, ReferenceImage, CurrentImage_left_right, H_left, H_right, H_left_right, i)
    
%% Initialize H to be used in the tracking function
if(tracking_param.changereference && tracking_param.changereference_key)
    switch tracking_param.initializer
        case "identity"
            Htrack = eye(3,3);
        case "hand_crafted"
            Htrack = eye(3,3);
            Htrack(:,end) = [-50;0;1];
        case "current_estimate"
            Htrack = inv(H_left(:,:,i)) * H_left_right(:,:,i-1) * H_right(:,:,i);
        case "prev_estimate"
            Htrack = H_left_right(:,:,i-1);
    end
else
    switch tracking_param.initializer
        case "current_estimate"
            Htrack = H_left_right(:,:,1) * H_right(:,:,i);
        case "prev_estimate"
            Htrack = H_left_right(:,:,i-1); %to pass it to TrackImageSL3
    end
end

%% Track
[H_left_right(:,:,i), WarpedImage_left_right, data] = ...
    TrackImageSL3(ReferenceImage, CurrentImage_left_right, Htrack, tracking_param);

%% Update H with the new estimate
if(tracking_param.changereference && tracking_param.changereference_key)
    % This computes H_LiRi (can be used to optimize H_LiRi)
    if tracking_param.use_optimized
        % This optimizes the current frame H_LiRi estimate (so it can be used
        % next time as initial estimate)
        H1_left_right = Htrack;
        H2_left_right = H_left_right(:,:,i);
        H_left_right(:,:,i) = get_avg_h(H1_left_right, H2_left_right);
    else
        % This optimizes the previous frame H_LiRi estimate, but it is
        % never used as initialization, it can be used in slam for example.
        H1_left_right = H_left_right(:,:,i-1);
        H2_left_right = H_left(:,:,i) * H_left_right(:,:,i) * inv(H_right(:,:,i));
        H_left_right(:,:,i-1) = get_avg_h(H1_left_right, H2_left_right);
    end
else
    % This computes H_L1Ri (can be used to optimize H_L1R1)
    H1_left_right = H_left_right(:,:,1);
    % H_L1Ri(since we don't change ref) * inverse of H_R1Ri = H_L1R1
    H2_left_right = H_left_right(:,:,i) * inv(H_right(:,:,i));
    % Update Homography between left and right cameras by averaging
    H_left_right(:,:,1) = get_avg_h(H1_left_right, H2_left_right);
end
end
