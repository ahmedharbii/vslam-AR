function [ReferenceImage, H, WarpedImage, norm_x] = track(tracking_param,ReferenceImage,CurrentImage,H,i)

if(tracking_param.changereference && tracking_param.changereference_key)
    Htrack = eye(3,3);
else
    if i <= 1
        Htrack = H(:,:,1);
    else
        Htrack = H(:,:,i-1); %to pass it to TrackImageSL3
    end
end

[H(:,:,i), WarpedImage, norm_x] = TrackImageSL3(ReferenceImage, CurrentImage, Htrack, tracking_param);

if(tracking_param.changereference && tracking_param.changereference_key)
    ReferenceImage = copy_image_data(ReferenceImage,CurrentImage);
end

end

