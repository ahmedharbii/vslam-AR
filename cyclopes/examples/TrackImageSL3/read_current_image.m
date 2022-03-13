function CurrentImage = read_current_image(capture_params, image_num_string)
    file_I = [capture_params.data_dir, capture_params.prefix, image_num_string,...
                    capture_params.suffix];
    % Read current image
    if(strcmp(capture_params.suffix, '.pgm'))
	    CurrentImage.I = imread(file_I);
    else
	    CurrentImage.Irgb = imread(file_I);
        CurrentImage.I = rgb2gray(CurrentImage.Irgb);
    end
end

