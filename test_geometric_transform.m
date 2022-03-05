kalb = imread("dog_grayscale_p.png");
kalb = imresize(kalb,[100, 100]);
original = zeros(480,632);
matchedPtsDistorted = [[0,0];[100,0];[100,100];[0,100]];
% matchedPtsOriginal = [
%     [198, 78.5];
%     [494, 173.5];
%     [354, 454.5];
%     [89, 351.5]];
matchedPtsOriginal = [
    [263, 68.5];
    [323, 217.5];
    [99, 285.5];
    [141, 128.5]];
[tform,inlierIdx] = estimateGeometricTransform2D(matchedPtsDistorted,matchedPtsOriginal,'projective');

inlierPtsDistorted = matchedPtsDistorted(inlierIdx,:);
inlierPtsOriginal  = matchedPtsOriginal(inlierIdx,:);

figure 
showMatchedFeatures(original,kalb,inlierPtsOriginal,inlierPtsDistorted)
title('Matched Inlier Points')

outputView = imref2d(size(original));
Ir = imwarp(kalb,tform,'OutputView',outputView);
figure 
imshow(Ir); 
title('Recovered Image');