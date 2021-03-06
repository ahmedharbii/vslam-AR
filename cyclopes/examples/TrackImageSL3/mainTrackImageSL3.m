%===================================================================================
%
% Copyright (C) 2010. All rights reserved.
%
% This sofware was developed at:
% CNRS/I3S
% 2000 Route des Lucioles
% 06903 Sophia Antipolis
%
% NAME: mainTrackImageSL3 - homography planar tracking algorithm
%
% PRE:   
%   capture_params - 
%						structure containing necessary info for the incoming images 
%						data_dir - directory where images are stored (can use environment variable DIR_DATA)
%   				prefix - filename prefix (i.e. 'pgm')
%						suffix - filename suffix (i.e. 'ima')
%   				first - the first image number 
%   				last - the last image number 
% 					string_size - the number string size
%						loadpolygon - bool to choose to load polygon from disk,
%						savepolygon - bool to choose to save polygon to disk
%
%   tracking_param - structure  containing info for tracking
%   				max_iter - the maximum number of iterations in the estimation loop
%						max_err - the minimum error threshold in the estimation loop
%						display - boolean to switch tracking display on or off
%           mestimator - boolean to switch mestimator off or on
%						esm - boolean to swich Efficient Second order Minimisation 
%
% POST:
%   H(:,:,i)- A list of Homographies for each image i.
%
% AUTHORS: Andrew Comport
% DATE: 1/1/2010
%	CONTACT: comport@i3s.unice.fr
%
%====================================================================================


function [H, data, change_ref_i, change_ref_x, change_ref_curr_img, change_ref_wrap_img_polygon] = mainTrackImageSL3(capture_params, tracking_param)

%% Setup debugging variables
global DEBUG_LEVEL_1;
global DEBUG_LEVEL_2;
global DEBUG_LEVEL_3;
DEBUG_LEVEL_1 = 0;
DEBUG_LEVEL_2 = 0;
DEBUG_LEVEL_3 = 0;

%% Testing Modes
overlap_image = false;
save_overlaped_image = false;
use_AR = false;
save_AR = false;
stereo = false;

%% AR
kalb = imread("dog_grayscale_p.png");
obj = readObj("models/fox.obj");
% display_obj(obj,"dog_grayscale_p.png")
w = 632;
h = 480;
a = 100 * pi / 180.0;
d = sqrt(w^2 + h^2);
f = 0.5 * d / tan(a/2);
camera_parameters = [
    [f, 0, w/2];
    [0, f, h/2];
    [0, 0, 1]];


if(nargin==0)
  disp('Launching test with default values...')
  test();
  return;
end

if(DEBUG_LEVEL_1)
	disp('TrackSL3');
	keyboard;
end

% Include project paths
addpath(sprintf('%s/include', capture_params.homedir));
include(capture_params.homedir);
addpath([pwd '\AR']);


close all;
%% Initialse - read reference image and select zone to track
% I can change the reference patch here or put inside the loop
% we want to copy the warped image not the current image when changing the
% reference patch
ReferenceImage = InitTrackImageSL3(capture_params);

close all;

%% Initialise Homography
%in calibration, no initalizing with identity
%intialising the homography matrix with 3x3 identity
H(:,:,1) = eye(3,3); 
H_left(:,:,1) = eye(3,3);
H_right(:,:,1) = eye(3,3);
H_left_right(:,:,1) = eye(3,3);
H_left_right(:,end,1) = [-50;0;1];
% H_left_right = [[1.1230, -0.2640, 1.6611];[0.0576, 0.9030, -10.9226];[ 0.0003, -0.0006, 0.9786]];

%% Initialize Setereo Variables
if stereo
    % read the first right image and make it the reference right image
    image_num_string = sprintf(['%0', num2str(capture_params.string_size), 'd'], capture_params.first);
    capture_params.data_dir = [pwd '\Versailles_canyon\Right\'];
    CurrentImage_right = read_current_image(capture_params, image_num_string);
    ReferenceImage_right = ReferenceImage;
    
    % Calculate Homography between left and right cameras
    % Transform the polygon from left reference image (left camera) to the right reference image
    tmp_tracking_param = tracking_param;
    tmp_tracking_param.changereference_key = 1;
    tmp_tracking_param.changereference = 1;
    tmp_tracking_param.initializer = "hand_crafted";
    tmp_tracking_param.use_optimized = true;
    [H_left_right, WarpedImage_left_right, data(1).left_right] =...
        track_left_right(tmp_tracking_param, ReferenceImage, CurrentImage_right,...
        H_left, H_right, H_left_right, 1);

    ReferenceImage_right = copy_image_data(ReferenceImage, CurrentImage_right, WarpedImage_left_right);
    data(1).left.norm_x = 0;
    data(1).right.norm_x = 0;
else
    data = [];
end

%% First Display
if(tracking_param.display)
    figure;
	scrsz = get(0,'ScreenSize');
	figure('Position',[scrsz(3)/4 scrsz(4)/2 scrsz(3)/2 scrsz(4)/2]);
	DrawImagePoly('Reference Image', 1, ReferenceImage.I, ReferenceImage.polygon);
    if stereo
        DrawImagePoly('Warped Current Image Left to Right', 2, CurrentImage_right.I, WarpedImage_left_right.polygon);
    end
end

%% Initially no ref change
tracking_param.changereference = 0;
tracking_param.changereference_key = 0; %to turn off the whole changing reference
tracking_param.changereference_thresh = 2000;

%% Loop
% Homography index
i=1;
% Loop through sequence
% store the itertion number and norm_x that had a reference image change
change_ref_i = [];
change_ref_x = [];
change_ref_curr_img = [];
change_ref_wrap_img_polygon = [];
%u can change the reference patch here to do the left right for example
for(k=capture_params.first+1:capture_params.last)
		i = i+1;
		image_num_string = sprintf(['%0', num2str(capture_params.string_size), 'd'], k);
        
        if stereo
            % track left-left , right-right , and left-right
            [ReferenceImage, ReferenceImage_right,...
                H_left, H_right, H_left_right,...
                data(i)...
                ]...
                = track_stereo(capture_params,image_num_string, tracking_param, i,...
                ReferenceImage, ReferenceImage_right, H_left, H_right, H_left_right, ...
                data(i-1).left.norm_x, data(i-1).right.norm_x);
            
        else
            if i == 2
                norm_residues = 0;
            else
                norm_residues = data(end).norm_residues;
            end
            % read the image
            CurrentImage = read_current_image(capture_params, image_num_string);
            % track
            [ReferenceImage, H, WarpedImage, data_i, tracking_param] =...
                track(tracking_param,ReferenceImage,CurrentImage,H,i,norm_residues);
            % for changing the reference patch for Question 6
            if(tracking_param.changereference && tracking_param.changereference_key)
                % these are for plotting purposes
                data_i.change_ref_i = i;
%                 change_ref_curr_img = [change_ref_curr_img CurrentImage.I];
%                 change_ref_wrap_img_polygon = [change_ref_wrap_img_polygon WarpedImage.polygon];
            else
                data_i.change_ref_i = -1;
            end
            data = [data data_i];
            assignin("base",capture_params.data_name, data)
%             if data_i.norm_x >= 200
%                 return;
%             end
        end
        
        if overlap_image
            % warp a 2d image inside a polygon defined patch.
    %         tform = projective2d(H(:,:,i)'); 
    %         Ir = imwarp(Ir_original, tform); 
            Ir = warp_2d_image(kalb, CurrentImage, WarpedImage.polygon);
        end
        
        if use_AR
            % estimate the 3d projection matrix
            proj_mat = calc_projection_matrix(camera_parameters, H(:,:,i));
        end

		if(tracking_param.display)
            fig_num = 1;
            if ~stereo
                % draw polygon, show warped image, and error image.
			    figure(fig_num); hold on;
                fig_num = fig_num + 1;
                DrawImagePoly('Warped Current Image', 1, CurrentImage.I, WarpedImage.polygon);
            end
            
            if overlap_image
                % overlay a 2d image on the polygon defined image patch
                overlaped_image = overlay_2_images(CurrentImage.I, Ir);
                figure(fig_num);
                fig_num = fig_num + 1;
                imshow(overlaped_image);
                if save_overlaped_image
                    Filename = sprintf('dog/test_%s.png', datestr(now,'mm-dd-yyyy HH-MM-SS'));
                    imwrite(overlaped_image,Filename);
                end
            end
            
            if use_AR
                % render a 3d object on the polygon defined image patch
                [x, y] = find(WarpedImage.I>0);
                img = render(CurrentImage.I, obj, proj_mat, 0.5 * (min(y) + max(y)), 0.5 * (min(x) + max(x)));
                
                if save_AR
                    % save the rendered image
                    Filename = sprintf('3d/test_%s.png', datestr(now,'mm-dd-yyyy HH-MM-SS'));
                    imwrite(img,Filename);
                end
    
                % show the rendered image
                figure(fig_num);
                fig_num = fig_num + 1;
                imshow(img);
            end

        end
end
return;

% Default test function if no values are given
function test()

tracking_params.max_iter = 100; %can stop tracking from here - 45
tracking_params.max_err = 200; %depends on the size of the patch, can do the average to be invariant on the patch size
tracking_params.max_x = 1e-4; %norm(x), when x comes small, I will stop - 1e-1
tracking_params.display = 1;
tracking_params.estimation_method = 2; % 1 = Reference Jacobian, 2 = Current Jacobian, 3 = ESM 
tracking_params.mestimator = 1;
tracking_params.robust_method='Tukey'; % Can be 'huber' or 'tukey' for the moment
tracking_params.scale_threshold = 2; % 1 grey level - try 2
tracking_params.size_x = 8; % number of parameters to estimate



% Change for your paths here
capture_params.homedir = [pwd '\cyclopes\']
%for the street:
% capture_params.data_dir = [pwd '\Versailles_canyon\Left\']
% capture_params.data_dir = [pwd '\Versailles_canyon\Right\']

%for underwater: 
capture_params.data_dir = [pwd '\IMAGES_smallRGB\'] 

%for the street:
% capture_params.prefix = 'ima';
% capture_params.suffix = '.pgm';

% for the underwater:
capture_params.prefix = 'img';
capture_params.suffix = '.png';

capture_params.string_size= 4; %4


capture_params.first = 160; %153
capture_params.last = 661;%661
capture_params.savepolygon = 0; % to save the polygon --> 1
capture_params.loadpolygon = 1; %to load the polygon --> 1


% tracking_params.estimation_method = 2; % 1 = Reference Jacobian, 2 = Current Jacobian, 3 = ESM 
% tracking_params.mestimator = 1;
% tracking_params.robust_method='Huber'; % Can be 'huber' or 'tukey' for the moment
% [H, data, change_ref_i, change_ref_x, change_ref_curr_img, change_ref_wrap_img_polygon] =...
%     mainTrackImageSL3(capture_params, tracking_params);
% assignin("base","data_1", data)
% capture_params.loadpolygon = 1;

% tracking_params.estimation_method = 3; % 1 = Reference Jacobian, 2 = Current Jacobian, 3 = ESM 
% tracking_params.mestimator = 1;
% tracking_params.robust_method='Huber'; % Can be 'huber' or 'tukey' for the moment
% [H, data, change_ref_i, change_ref_x, change_ref_curr_img, change_ref_wrap_img_polygon] =...
%     mainTrackImageSL3(capture_params, tracking_params);
% assignin("base","data_2", data)

% tracking_params.estimation_method = 2; % 1 = Reference Jacobian, 2 = Current Jacobian, 3 = ESM 
% tracking_params.mestimator = 1;
% tracking_params.robust_method='Tukey'; % Can be 'huber' or 'tukey' for the moment
% [H, data, change_ref_i, change_ref_x, change_ref_curr_img, change_ref_wrap_img_polygon] =...
%     mainTrackImageSL3(capture_params, tracking_params);
% assignin("base","data_3", data)

tracking_params.estimation_method = 3; % 1 = Reference Jacobian, 2 = Current Jacobian, 3 = ESM 
tracking_params.mestimator = 1;
tracking_params.robust_method='Tukey'; % Can be 'huber' or 'tukey' for the moment
capture_params.data_name = "data_4";
[H, data, change_ref_i, change_ref_x, change_ref_curr_img, change_ref_wrap_img_polygon] =...
    mainTrackImageSL3(capture_params, tracking_params);
assignin("base","data_4", data);
% figure(2)
% plot();
% hold on;

scatter(change_ref_i, change_ref_x, 10,'red','filled',"o");
% 
% for i=1:size(change_ref_x,1)
%     figure(3);
%     w = waitforbuttonpress;
%     sz1 = uint16(size(change_ref_curr_img, 2) / 3);
%     sz = uint16(size(change_ref_wrap_img_polygon, 2) / 3);
%     DrawImagePoly('Warped Current Image', 1, change_ref_curr_img(:,((i-1)*sz1 + 1):i*sz1), change_ref_wrap_img_polygon(:,((i-1)*sz + 1):i*sz));
%     title(["change at frame=" change_ref_i(i) "x=" change_ref_x(i)])
% end

return;