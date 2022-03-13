%===================================================================
%
% Copyright (C) 2010. All rights reserved.
%
% This sofware was developed at:
% CNRS/I3S
% 2000 Route des Lucioles
% 06903 Sophia Antipolis
%
% NAME: TrackImageSL3 - Non-linear iterative estimation of 3x3 planar 
%												homography between reference and current image
% PRE: - Reference image with I, P, JI, JP
%			 - Current image with I
%			 - Current estimate/prediction of Homographie H
% POST:  Hnew - New esimate of Homography
% AUTHOR: Andrew Comport
% DATE: 1/1/08
%	CONTACT: comport@i3s.unice.fr
%
%===================================================================


function [Hnew, WarpedImage, data] = TrackImageSL3(ReferenceImage, CurrentImage, H, tracking_param)

global DEBUG_LEVEL_2;
if(DEBUG_LEVEL_2)
	disp('TrackingSL3');
	keyboard;
end

% Initialise Homography
Hnew = H;
residues = 1000000; %starting with big residue
iter = 0;
x = 10000000; % So that initially the norm(x) in the while loop is large 

% Iterative minimization
%while(YOUR STOPPING CRITERION HERE)% 
% while(1)
while(iter< tracking_param.max_iter && norm(x) > tracking_param.max_x && norm(residues) > tracking_param.max_err) %& norm(x) < tracking_param.max_x & norm(residue) < tracking_param.max_err)
		% Current patch
    WarpedImage = WarpImageSL3(CurrentImage, ReferenceImage, Hnew);    

	  % Patch error/residue in vector form	-- changed it from residues to
      % residue
    residues = double(ReferenceImage.I(WarpedImage.index)) - WarpedImage.I(WarpedImage.index);
%     disp('residues:')
%     disp(norm(residues))
%     disp('iter')
%     disp(iter)
%     disp('max_x')
%     disp(norm(x))

		switch tracking_param.estimation_method

			case 1
				% If M-estimator is used then dont use pre computed Jacobian
				if(tracking_param.mestimator == 1)
					WarpedImage.J = ReferenceImage.J(WarpedImage.visibility_index,:); 
                end
				%Pseudo inverse precalculated if no mestimator
				[x, weights] = Estimate(WarpedImage, ReferenceImage, residues, tracking_param);

		    case 2
				% Compute Current Jacobian
				[WarpedImage.J, WarpedImage.JI] = JacobianImageSL3(WarpedImage.I, WarpedImage.P, WarpedImage.index);
				[x, weights] = Estimate(WarpedImage, ReferenceImage, residues, tracking_param);

			case 3 % If second order minimisation (ESM) recompute current image gradient at each iteration
				% Note esm jacobian stored in WarpedImage
			  WarpedImage.J = JacobianImageESMSL3(WarpedImage, ReferenceImage); 
				[x, weights] = Estimate(WarpedImage, ReferenceImage, residues, tracking_param);

			otherwise
				error('Tracking estimation method does not exist');

        end

		% Compute unknown parameters x , A=3*3
    A = [x(5),x(3),x(1); x(4),-x(5)-x(6),x(2); x(7),x(8),x(6)]; 
    Hnew = Hnew*expm(A); % Compute the update of the Homography matrix using the exponential map

		if(tracking_param.display)
			figure(1); 
			subplot(2,2,4); imagesc(WarpedImage.I); colormap(gray); title('Warped Image'); axis off;  
			W = zeros(ReferenceImage.sIv, ReferenceImage.sIu);
			W(WarpedImage.index) = weights;
			subplot(2,2,3); imagesc(W); colormap(gray); title('Weight Image'); axis off;
			subplot(2,2,2); image(abs(WarpedImage.I-double(ReferenceImage.I)).*WarpedImage.Mask.*W); 
			colormap(gray); 	title('Error Image'); 	axis off;  
			%keyboard;
        end

		iter = iter+1;

		if(tracking_param.display)
            disp('residues:')
            disp(norm(residues))
            disp('iter')
            disp(iter)
            disp('max_x')
            disp(norm(x))

        end
end

if(tracking_param.display)

	% Transform polygon
    %Augmented Reality can be implemented here and can use Matlab library
    %To change the shape of polygon
	WarpedImage.polygon = Hnew*ReferenceImage.polygon; 
	WarpedImage.polygon = WarpedImage.polygon./repmat(WarpedImage.polygon(3,:),3,1);
    
    %To display this in the loop
% 	norm(x)
% 	iter
%     norm(residues)


end

data.x = x;
data.norm_x = norm(x);
data.residues = residues;
data.norm_residues = norm(residues);
data.iter = iter;
data.currentImage = CurrentImage;
data.WarpedImage = WarpedImage.polygon;
return

