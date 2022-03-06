%% find 3d projection matrix
function result = calc_projection_matrix(camera_parameters, homography)
    %From the camera calibration matrix and the estimated homography
    %compute the 3D projection matrix
    % Compute rotation along the x and y axis as well as the translation

    homography = homography * -1;
    rot_and_transl = camera_parameters \ homography;
    col_1 = rot_and_transl(:, 1);
    col_2 = rot_and_transl(:, 2);
    col_3 = rot_and_transl(:, 3);
    % normalise vectors
    l = sqrt(norm(col_1, 2) * norm(col_2, 2));
    rot_1 = col_1 / l;
    rot_2 = col_2 / l;
    translation = col_3 / l;
%     translation(1,1) = -1 * translation(1,1);
%     translation(3,1) = -1 * translation(3,1);
    % compute the orthonormal basis
%     c = rot_1 + rot_2;
%     p = cross(rot_1, rot_2);
%     d = cross(c, p);
%     rot_1 = (c / norm(c) + d / norm(d)) *  (-1 / sqrt(2));
%     rot_2 = (c / norm(c) - d / norm(d)) *  (1 / sqrt(2));
    rot_3 = cross(rot_1, rot_2);
    
    % finally, compute the 3D projection matrix from the model to the current frame
    projection = [rot_1, rot_2, rot_3, translation];
    result = camera_parameters * projection;
    return;