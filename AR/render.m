%% render .obj file in the image using projection matrix
function img = render(img, obj, projection, h, w)
    
% Render a loaded obj model into the current video frame

    vertices = obj.v;
    scale_matrix = eye(3) * 1;

    rot_x = [[rotx(70); [0, 0, 0]], [0; 0; 0; 1]];
    rot_y = [[roty(0); [0, 0, 0]], [0; 0; 0; 1]];
    rot_z = [[rotz(0); [0, 0, 0]], [0; 0; 0; 1]];
    rot = rot_x * rot_y * rot_z;
%     h = size(model,1);
%     w = size(model,2);
%     h = 200;
%     w = 300;
    
    % find center of 3d model to use it to translate its center to patch
    % center
    points = [];
    for i=1:size(obj.f.v,1)
        face_vertices = obj.f.v(i,:);
        for v_idx=1:size(face_vertices,2)
            vertex = face_vertices(v_idx);
            points = [points; vertices(vertex,:)];
        end
        points = points * scale_matrix;

        % render model in the middle of the reference surface. To do so,
        % model points must be displaced
    end
    center_x = 0.5 * (abs(min(points(:, 1))) + abs(max(points(:, 1))));
    center_y = 0.5 * (abs(min(points(:, 2))) + abs(max(points(:, 2))));


    for i=1:size(obj.f.v,1)
        points = [];
        face_vertices = obj.f.v(i,:);
        for v_idx=1:size(face_vertices,2)
            vertex = face_vertices(v_idx);
            points = [points; vertices(vertex,:)];
        end
        points = points * scale_matrix;
        points = [points, ones(size(points,1), 1)];
        points = (rot * points')';

        % render model in the middle of the reference surface. To do so,
        % model points must be displaced
        tmp_points = [];
        for p_idx=1:size(points,1)
            p = points(p_idx,:);
            tmp_points = [tmp_points; [p(1) + w + center_x, p(2) + h - center_y, p(3), 1]];
        end
        points = tmp_points;

        % Prespective Transform
        %=======================

    %     tform = geometricTransform3d(project_func);
    %     tform = projective2d(projection);
    %     rot = projection(:,1:3)';
    %     trans = projection(:,4)';
    %     tform = rigid3d(single(rot), single(trans));
    %     ptCloud1 = pointCloud(single(points));
    %     dst = pctransform(ptCloud1, tform);
        
        % construct the affine projection transform
        hom_projection = [projection; [0, 0, 0, 1]];
        tform = affine3d(hom_projection');
        
        % apply the transform to the 3d model points
        [U1,V1,W1] = transformPointsForward(tform,points(1,1),points(1,2), points(1,3));
        [U2,V2,W2] = transformPointsForward(tform,points(2,1),points(2,2), points(2,3));
        [U3,V3,W3] = transformPointsForward(tform,points(3,1),points(3,2), points(3,3));
        dst = [
            [U1/W1,V1/W1];
            [U2/W2,V2/W2];
            [U3/W3,V3/W3]];
%         dst = hom_projection * points';
        imgpts = int32(dst);

        % fill the pixels in the image correspoinding to the 3d model
        % projected image.
        mask = roipoly(img,imgpts(:,1),imgpts(:,2));
        img(mask) = 255;

    end
    return;