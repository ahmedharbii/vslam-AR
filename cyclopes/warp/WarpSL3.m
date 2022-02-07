%===============================================================
%
% Copyright (C) 2010. All rights reserved.
%
% This sofware was developed at:
% CNRS/I3S
% 2000 Route des Lucioles
% 06903 Sophia Antipolis
%
% NAME: WarpSL3
% METHOD: Homographic warping of image points with normalisation
% PRE: Points to be warped, Homography.
% POSE: Return warped points in indexed vector form
% AUTHOR: Andrew Comport
%	CONTACT: comport@i3s.unice.fr
%
%===============================================================

function WarpedImage = WarpSL3(ReferenceImage, H)

global DEBUG_LEVEL_3;
if(DEBUG_LEVEL_3)
	disp('WarpSL3');
	keyboard;
end

% keyboard; % enter debug mode here - use dbquit to exit debug mode

%TODO : 
% 1. Warp patch coordinates (u, v, w); "multiply h by the coordinates"
%u v 1
% points = [ReferenceImage.P.U(:) ReferenceImage.P.V(:) ones(size(ReferenceImage.P.U(:)))]';
warped_points = [ReferenceImage.P.U(ReferenceImage.index) ReferenceImage.P.V(ReferenceImage.index) ones(size(ReferenceImage.P.U(ReferenceImage.index)))]';
warped_points = H*warped_points; %matrix multiplication ???

% 2. Normalise coordinates so that w = 1; 
%.index gets the points to my patch
%pixel warped
% warped_points = H*[ReferenceImage.P.U(ReferenceImage.index) ReferenceImage.P.V(ReferenceImage.index) ones(size(ReferenceImage.P.U(ReferenceImage.index)))]';
warped_points_normalized = warped_points./warped_points(3,:); %to normalise the last column
% 3. Check if the warped pixels fall inside the image and update indexes accordingly
u = warped_points_normalized(1,:);
v = warped_points_normalized(2,:);
w = warped_points_normalized(3,:);
u = u./w;
v = v./w;

%determine pixels inside of image and update indexex
WarpedImage.visibility_index = find(u<=ReferenceImage.sIu+1 & u>=1 & v<=ReferenceImage.sIv+1 & v>=1);
WarpedImage.index = ReferenceImage.index(WarpedImage.visibility_index);
% find(warped_points_normalized, points)

% Stored in WarpedImage.P.U and WarpedImage.P.V which are images of size:
%stored in full image size matrix
WarpedImage.P.U = zeros(size(ReferenceImage.I));
WarpedImage.P.V = zeros(size(ReferenceImage.I));
%this is added:
WarpedImage.P.U(WarpedImage.index) = u(WarpedImage.visibility_index);
WarpedImage.P.V(WarpedImage.index) = v(WarpedImage.visibility_index);

% Propagate and update Mask and indexes
WarpedImage.Mask = zeros(ReferenceImage.sIv,ReferenceImage.sIu);
WarpedImage.Mask(WarpedImage.index) = 1;

return
