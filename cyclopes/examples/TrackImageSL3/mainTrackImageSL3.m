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


function [H] = mainTrackImageSL3(capture_params, tracking_param)

% Setup debugging variables
global DEBUG_LEVEL_1;
global DEBUG_LEVEL_2;
global DEBUG_LEVEL_3;
DEBUG_LEVEL_1 = 0;
DEBUG_LEVEL_2 = 0;
DEBUG_LEVEL_3 = 0;

%for reference patch changing - Question 6
tracking_param.changereference = 1; 
tracking_param.changereference_thresh = 1000;

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

close all;
% Initialse - read reference image and select zone to track
% I can change the reference patch here or put inside the loop
% we want to copy the warped image not the current image when changing the
% reference patch
ReferenceImage = InitTrackImageSL3(capture_params);
close all;

if(tracking_param.display)
	scrsz = get(0,'ScreenSize');
	figure('Position',[scrsz(3)/4 scrsz(4)/2 scrsz(3)/2 scrsz(4)/2]);
	DrawImagePoly('Reference Image', 1, ReferenceImage.I, ReferenceImage.polygon);
end

% Initialise Homography
%in calibration, no initalizing with identity
H(:,:,1) = eye(3,3); %intialising the homography matrix with 3x3 identity

% Homography index
i=1;
% Loop through sequence
% Store the x values through the iterations
all_x = [];
%u can change the reference patch here to do the left right for example
for(k=capture_params.first+1:capture_params.last)
		i = i+1;
	
		image_num_string = sprintf(['%0', num2str(capture_params.string_size), 'd'], k);
		file_I = [capture_params.data_dir, capture_params.prefix, image_num_string, capture_params.suffix];

		% Read current image
		if(strcmp(capture_params.suffix, '.pgm'))
			CurrentImage.I = imread(file_I);

		else
			CurrentImage.Irgb = imread(file_I);
		  CurrentImage.I = rgb2gray(CurrentImage.Irgb);
        end
        
        if(tracking_param.changereference)
            Htrack = eye(3,3);
        else
            Htrack = H(:,:,i-1); %to pass it to TrackImageSL3
        end

		% Iterative non-linear homography estimation
    [H(:,:,i), WarpedImage, norm_x] = TrackImageSL3(ReferenceImage, CurrentImage, Htrack, tracking_param);
		H(:,:,i) %for displaying
        % for changing the reference patch for Question 6
        all_x = [all_x norm_x];
        tracking_param.changereference = change_ref_or_not(norm_x, tracking_param);
        if(tracking_param.changereference)
            ReferenceImage.I = CurrentImage.I;
            %ReferenceImage.Irgb = CurrentImage.Irgb;
            ReferenceImage.polygon = WarpedImage.polygon;
            ReferenceImage.index = WarpedImage.index;
            ReferenceImage.Mask = WarpedImage.Mask;
        end
	
		if(tracking_param.display)
			figure(1); hold on;	
			DrawImagePoly('Warped Current Image', 1, CurrentImage.I, WarpedImage.polygon);
        end

end

return;


% function to decide whether to change ref image or not
function [change] = change_ref_or_not(norm_x, tracking_param)
change = false;
if norm_x > tracking_param.changereference_thresh
    change = true
end
return


% Default test function if no values are given
function test()

tracking_params.max_iter = 165; %can stop tracking from here - 45
tracking_params.max_err = 400; %depends on the size of the patch, can do the average to be invariant on the patch size
tracking_params.max_x = 1e-4; %norm(x), when x comes small, I will stop - 1e-1
tracking_params.display = 1;
tracking_params.estimation_method = 3; % 1 = Reference Jacobian, 2 = Current Jacobian, 3 = ESM 
tracking_params.mestimator = 1;
tracking_params.robust_method='tukey'; % Can be 'huber' or 'tukey' for the moment
tracking_params.scale_threshold = 2; % 1 grey level - try 2
tracking_params.size_x = 8; % number of parameters to estimate



% Change for your paths here
capture_params.homedir = [pwd '\cyclopes\']
%for the street:
% capture_params.data_dir = '/MIR Erasmus/VSLAM/Versailles_canyon/Right/'
capture_params.data_dir = [pwd '\Versailles_canyon\Left\']
%for underwater: 
% capture_params.data_dir = '/MIR Erasmus/VSLAM/IMAGES_smallRGB/'
%capture_params.data_dir = [getenv('DIR_DATA'), '/../data/Versailles/Versailles_canyon/Left/']; 
%capture_params.homedir = getenv('DIR_CYCLOPES'); 

%for the street:
capture_params.prefix = 'ima';
capture_params.suffix = '.pgm';

% for the underwater:
% capture_params.prefix = 'img'
% capture_params.suffix = '.png'

capture_params.string_size= 4; %4
capture_params.first = 50; %1
capture_params.last = 100;
capture_params.savepolygon = 1; % to save the polygon --> 1
capture_params.loadpolygon = 0; %to load the polygon --> 1


[H] = mainTrackImageSL3(capture_params, tracking_params);

return;
