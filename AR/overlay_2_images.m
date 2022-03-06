% overlay foreground_image on background_image
% where foreground is zeroes at pixels that are related to backround.
function overlaped_image = overlay_2_images(background_image, foreground_image)
    [x, y] = find(foreground_image>0);
    overlaped_image = background_image;
    for i=1:size(x, 1)
        overlaped_image(x(i), y(i)) = foreground_image(x(i), y(i));
    end
return;