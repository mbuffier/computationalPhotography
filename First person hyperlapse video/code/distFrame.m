function r = distFrame(folderName)

% create a folder with the images to pally the sfm algorithm (without
% redondant frames and only 1 frame over 3)

numberOfFrame = fromVideoToImages('paris.m4v', 3, folderName) ;

%************** Step 1 : Compute the distances for each frame ***********
distFrame = zeros(numberOfFrame, numberOfFrame) ;
for i = 41:numberOfFrame-40
    tic ;
    index = i-1 ;
    imgIname = ['SfM/' folderName '/test0' num2str(index) '.jpg'] ;
    thisImage = imread(imgIname) ;
    for j = i-40:i+40
        index2 = j-1 ;
        imgJname = ['SfM/' folderName '/test0' num2str(index2) '.jpg'] ;
        thisImage2 = imread(imgJname) ;
        
        dist = computeDistColor(thisImage, thisImage2) ;
        distFrame(i,j) = dist ;
    end
    toc ;
end
imshow(distFrame, []) ;
imwrite(distFrame,'distFrameTest.jpg') ;
%distFrame = imread('distFrame.jpg') ;

%************** Step 2 : Determine the similar frame anf visualize them ***********

similarFrames = sum(distFrame, 2) ;

% create the mask 
mask1 = similarFrames < 16000 ;
mask2 = similarFrames > 1000 ;
mask = mask1 & mask2 ;

% visualize the result 
distFrameColor = visualizeColor(mask, distFrame) ;
imshow(distFrameColor) ;

%************** Step 3 : Remove the frames from the folder ***********
indexNew = 0 ;
for i=1:numberOfFrame
    indexOld = i-1 ;
    imgName = ['SfM/' folderNameInput '/test0' num2str(indexOld) '.jpg'] ;
    thisImage = imread(imgName) ;
    
    delete(imgName) ;
    
    if mask(i)==0 
       newimgName = ['SfM/' folderNameInput '/test0' num2str(indexNew) '.jpg'] ;
       indexNew = indexNew+1 ;
       imwrite(thisImage, newimgName) ;
    end
    
end
end

% compute the color distance
function dist = computeDistColor(img1, img2)
distR = computeDist1Chanel(img1(:,:,1), img2(:,:,1)) ;
distG = computeDist1Chanel(img1(:,:,2), img2(:,:,2)) ;
distB = computeDist1Chanel(img1(:,:,3), img2(:,:,3)) ;

dist = sqrt(distR^2+distG^2+distB^2) ;
end

% compute the distance for one chanel image
function dist = computeDist1Chanel(img1, img2)
diff = (img1-img2).^2 ;
dist = sum(diff(:)) ;
dist = dist/(size(img1,1)*size(img1,2)) ;
end

% color the row and column indicated by "mask" in red 
function distFrameColor = visualizeColor(mask, distFrame)

numberOfFrame = size(mask, 1) ;
distFrameColor = cat(3,distFrame, distFrame,distFrame ) ;
red = reshape([255 0 0], [1,1,3]) ;

% create the row and col red 
thisRowRed = repmat(red, [1, numberOfFrame, 1]) ;
thisColRed = repmat(red, [numberOfFrame, 1, 1]) ; 

% if mask == 1, change the color 
for i = 1: numberOfFrame
    if mask(i)==1
        distFrameColor(i,:,:) = thisRowRed ;
        distFrameColor(:,i,:) = thisColRed ;
    end 
end

end