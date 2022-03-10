function CurrentImage = read_current_image(file_I, capture_params)
    % Read current image
    if(strcmp(capture_params.suffix, '.pgm'))
	    CurrentImage.I = imread(file_I);
    else
	    CurrentImage.Irgb = imread(file_I);
        CurrentImage.I = rgb2gray(CurrentImage.Irgb);
    end
end

