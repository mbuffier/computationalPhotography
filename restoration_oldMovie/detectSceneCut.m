function [frameChange, result] = detectSceneCut(matrixImage)
% take a matrix of images and gives the resulting matrix with the incription of the
% scene cut on it

numFrame = size(matrixImage,3) ;
frameDiff = zeros(numFrame-1,1) ;

for i=2:numFrame
    % get the 2 frames 
    img1 = matrixImage(:,:,i-1) ;
    img2 = matrixImage(:,:,i) ;
    
    % take the edge from those 2
    edge1 = im2double(edge(img1)) ;
    edge2 = im2double(edge(img2)) ;

    % dilate the edge using a gaussian filter
    edge12 = imgaussfilt(edge1,30) ;
    edge22 = imgaussfilt(edge2,30) ;
    
    % compute the squared differences 
    squaredDiff = (edge12 - edge22).^2 ;
    
    frameDiff(i-1,1) = sum(squaredDiff(:)) ;
end

% normalized the result 
frameDiff = frameDiff./max(frameDiff(:)) ;

% detect the frame changes 
frameChange = find(frameDiff>0.5) ;
if isempty(frameChange)
    frameChange =0; 
end

% insert the text on the image 
for i=1:size(frameChange,1)
    img = matrixImage(:,:,frameChange(i)+1) ;
    newIm = insertText(img,[100 100],'Scene Cut','FontSize',50) ;
    newIm = rgb2gray(newIm) ;
    matrixImage(:,:,frameChange(i)+1) = newIm ;
end

result = matrixImage ;

end