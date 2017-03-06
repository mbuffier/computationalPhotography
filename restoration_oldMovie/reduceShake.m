function result = reduceShake(matrixImage,frameChange)

[M,N, numTotalFrame] = size(matrixImage) ;
result = zeros(M,N,numTotalFrame) ;

% in case we only take part of the video
if frameChange == 0
    frameChange = [0 ; numTotalFrame] ;
    numSeq = 1 ;
else
    frameChange = [0; frameChange; numTotalFrame] ;
    numSeq = size(frameChange,1)-1 ;
end

for i=1:numSeq
    % extract the images of the sequence
    imageSeq = matrixImage(:,:,frameChange(i)+1:frameChange(i+1)) ;
    
    % apply the flicker reduce effect
    imageSeqCorrect = reduceShakeSeq(imageSeq) ;
    
    result(:,:,frameChange(i)+1:frameChange(i+1)) = imageSeqCorrect;
end

end

function result = reduceShakeSeq(matrixImage)

[M,N, nbFrame] = size(matrixImage) ;
threshold = 0.01;
result = zeros(M,N, nbFrame, 'uint8') ;
result(:,:,1) = matrixImage(:,:,1) ;

oldTrans = eye(3) ;
for i=2:nbFrame
    % compute the descriptor to
    [des1, locs1] = surf(matrixImage(:,:,i-1)) ;
    [des2, locs2] = surf(matrixImage(:,:,i)) ;
    
    % find the pose of the new array
    [pos1, pos2] = connectedSIFT(locs1, locs2, des1, des2, threshold) ;
    %displayMatch(pos1,pos2,oldFrame,matrixImage(:,:,i)) ;
    
    % only take pos with don't move a lot (backgroup point)
    [pos1Small, pos2Small] = smallMoveFeatures(pos1, pos2) ;
    %displayMatch(pos1Small,pos2Small,oldFrame,matrixImage(:,:,i));
    test = checkCollinear(pos2Small) ;
    
    % if there are not enough feature points or point in pos2Small are non collinear we dont compute the
    % transformation
    if test == 0
        transf = eye(1) ;
    else
        %calcul the best homographie between the features points using ransac
        transf = ransac(pos1Small.', pos2Small.') ;
    end    
    % correct with respect to the first one 
    transf = transf*oldTrans ;
    
    % correct the second frame by applying the inverse transformation
    newFrame = computeCorrectFrame(matrixImage(:,:,i), transf) ;
    
    % croop the result
    result(:,:,i) = newFrame ;
    oldTrans = transf ;
end
result = uint8(result) ;
end

function  test =  calcBestSimilarity(pos1Small, pos2Small)
t = fitgeotrans(pos1Small,pos2Small, 'NonreflectiveSimilarity') ;
test = t.T.' ;
end

function [descriptors, locs] = surf(image)
points = detectSURFFeatures(image);
[descriptors, valid_points] = extractFeatures(image, points);
locs = [valid_points.Location];
end

function [pos1, pos2] = connectedSIFT(locs1, locs2, descrip1, descrip2, threshold)
[M1, ~] = size(descrip1) ;
[M2, ~] = size(descrip2) ;
numbMatch = 1 ;

pos1 = zeros(M1,2) ;
pos2 = zeros(M2,2) ;

for i = 1:M1
    % take one descriptor in image 1
    thisDescrip = descrip1(i,:) ;
    thisDescrip = repmat(thisDescrip, [M2, 1]) ;
    
    % compute the ssd with the descriptors of im2
    dist = sum((descrip2-thisDescrip).^2,2) ;
    distMin = min(dist(:)) ;
    indexMin = (find(dist==distMin)) ;
    
    distMin2 = min(dist(1:end ~= indexMin)) ;
    
    % if the distance between the 2 maximum is above the threshold, it's a
    % match
    if (distMin2-distMin > threshold)
        % it's a match
        pos1(numbMatch,:) = uint16(locs1(i, 1:2)) ;
        pos2(numbMatch,:) = uint16(locs2(indexMin, 1:2)) ;
        numbMatch = numbMatch + 1 ;
    end
end

pos1 = pos1(1:(numbMatch-1),:) ;
pos2 = pos2(1:(numbMatch-1),:) ;
end

% check if at least vectors are non collinear in a set
function test = checkCollinear(points)
test=0 ;
% unique points in the set 
[uniqueP2,~,~] = unique(points,'rows') ;

% check the director coefficiants
coeff = uniqueP2(:,1)./uniqueP2(:,2) ;

% number of non collinear points 
[~,indicesNC,~] = unique(coeff,'rows') ;

M = size(indicesNC, 1) ;

if M > 1
    test=1 ;
end
end

% only take points in the background
function [pos1Result, pos2Result] = smallMoveFeatures(pos1, pos2)
% we compute the distance on x and y for the 2 sets
dist = sum((pos1-pos2).^2,2) ;

pos1Result = zeros(size(pos1)) ;
pos2Result = zeros(size(pos2)) ;
numberMatch = 0 ;

for i=1:size(pos2,1)
    if dist(i) < 20
        pos1Result(numberMatch+1,:) = pos1(i,:) ;
        pos2Result(numberMatch+1,:) = pos2(i,:) ;
        numberMatch = numberMatch+1 ;
    end
end
pos1Result = pos1Result(1:numberMatch,:) ;
pos2Result = pos2Result(1:numberMatch,:) ;
end

% apply the ransac algorithm to find the best transformation
function TFinal = ransac(pts1Cart, pts2Cart)
sizeFeatVec = size(pts1Cart,2) ;

% 20 iterations to find the best transformation
numIte = 100 ;
% initialization
match1 = zeros(2*numIte, sizeFeatVec) ;
match2 = zeros(2*numIte, sizeFeatVec) ;
numMatch = zeros(numIte,1) ;
indiceMatch = 1 ;

% compute the homography using random feature points
for i=1:numIte
    % randomly select 2 non collinear features
    [selectFeat1, selectFeat2] = selectRandomF(pts1Cart', pts2Cart') ;

    % compute the best homography for those set of points
    TEst = calcBestSimilarity(selectFeat1, selectFeat2) ;
    
    % compute the number of inliers for these trans in this set
    [numMatch(i,1),match1(indiceMatch:indiceMatch+1,:) , match2(indiceMatch:indiceMatch+1,:)] ...
        = numberInlier(TEst,pts1Cart, pts2Cart) ;
    indiceMatch = indiceMatch+2 ;
end
% find the transformation with the most inlier
indx = find(numMatch==max(numMatch(:))) ;

% get back the inlier from the distribution
newFeatPoint1 = match1(2*indx(1)-1:2*indx(1),1:max(numMatch(:))) ;
newFeatPoint2 = match2(2*indx(1)-1:2*indx(1),1:max(numMatch(:))) ;

% if not at least 2 non collinear points in newFeatPoint2, we compute the
% total transformation
if checkCollinear(newFeatPoint2.')
    TFinal = calcBestSimilarity(newFeatPoint1.', newFeatPoint2.') ;
else
    TFinal = calcBestSimilarity(pts1Cart.', pts2Cart.') ;
end
end

% select 2 random matching pairs with 2 non collinear point in pts2Cart
function [selectFeat1, selectFeat2] = selectRandomF(pts1Cart, pts2Cart)
selectFeat1 = zeros(2,2) ;
selectFeat2 = zeros(2,2) ;

% unique points in the set 
[uniqueP2,indices,~] = unique(pts2Cart,'rows') ;

% check the director coefficiants in the unique points set to avoid
% collinearity 
coeff = uniqueP2(:,1)./ uniqueP2(:,2) ;
[~,indicesNC,~] = unique(coeff,'rows') ;

% choose among non collinear vector
M = size(indicesNC, 1) ;
randomIndice1 = randi([1 M]) ;

% first points 
selectFeat1(1,:) = pts1Cart(indices(indicesNC(randomIndice1)),:) ;
selectFeat2(1,:) = pts2Cart(indices(indicesNC(randomIndice1)),:) ;

indicesNC(randomIndice1) = [] ;
randomIndice2 = randi([1 M-1]) ;
%second points
selectFeat1(2,:) = pts1Cart(indices(indicesNC(randomIndice2)),:) ;
selectFeat2(2,:) = pts2Cart(indices(indicesNC(randomIndice2)),:) ;
end


function [nbMatch, match1, match2] = numberInlier(hom,pts1Cart, pts2Cart)
N = size(pts1Cart,2) ;

% homogenous coordinates
pts1Cart = [pts1Cart ; zeros(1,N)] ;
pts2Cart = [pts2Cart ; zeros(1,N)] ;

% difference between the set and its transform
diffVec = pts1Cart-hom*pts2Cart ;

% look how many points are above the threshold
ssdVals = sqrt(sum(diffVec.^2, 1)) ;
mask = ssdVals<1 ;
nbMatch = sum(ssdVals<1) ;

match1 = zeros(2, N) ;
match2 = zeros(2, N) ;
index = 1 ;

% store the inliers
for i=1:N
    if mask(1,i)==1
        match1(:,index) = pts1Cart(1:2,i) ;
        match2(:,index) = pts2Cart(1:2,i) ;
        index = index+1 ;
    end
end

end

function newFrame = computeCorrectFrame(img, hom)

[M,N,~] = size(img) ;
newFrame = zeros(M,N) ;

for i=1:M
    for j = 1:N
        % transform the position of the pixel under the best homographie
        thisPixelPos = [j,i,1] ;
        % find the new position
        newPos = hom*thisPixelPos.' ;
        
        newPos = newPos(:,:)./newPos(3,:) ;
        %if the position is in the frame, we fill the new frame with the pixel which erase the transformation
        if (newPos(1) > 1 && newPos(1) < N && newPos(2)>1 && newPos(2)< M)
            % interpolation between the 4 points around
            i1 = floor(newPos(2)) ;
            j1 = floor(newPos(1)) ;
            
            deltaI = newPos(2) - i1 ;
            deltaJ = newPos(1) - j1 ;
            
            % interpolation of the pixel values
            thisPixel = deltaI*deltaJ*img(i1+1,j1+1)+ deltaI*(1-deltaJ)*img(i1+1,j1)+ ...
                (1-deltaI)*deltaJ*img(i1,j1+1) + (1-deltaI)*(1-deltaJ)*img(i1,j1);
            newFrame(i,j) = thisPixel ;
        end
    end
end
newFrame = uint8(newFrame) ;
end

% display the features points in the 2 frames
function r = displayMatch(pos1,pos2,im1,im2)
[M1, N1] = size(im1) ;
[M2, ~] = size(im2) ;

fig1 = figure,
set(fig1,'Position',[0 0 700 900]);

imResul = [im1; zeros(M2-M1,N1)] ;
imResul = [uint8(imResul) uint8(im2)] ;

subplot(1,1,1),
imshow(imResul), hold on;
numMatchbook = size(pos1,1) ;

for thisMatch = 1:numMatchbook
    posXScene = N1 +  pos2(thisMatch,2) ;
    plot([pos1(thisMatch,2),posXScene],[pos1(thisMatch,1) pos2(thisMatch,1)],'g-');
    hold on;
end;
hold off

end
