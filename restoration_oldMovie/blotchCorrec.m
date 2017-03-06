function resultMatrix = blotchCorrec( matrixImage )

[M, N, nbFrame] = size(matrixImage) ;
resultMatrix = zeros(M,N, nbFrame) ;

% fill the border 
resultMatrix(:,:,1) = matrixImage(:,:,1) ;
resultMatrix(:,:,nbFrame) = matrixImage(:,:,nbFrame) ;

% bigger image to avoid 0 indices in the distance computation
matrixImage2 = padarray(matrixImage, [1 1 0], 'replicate') ;

% 2 thresholds for the function
threshold1 = 0.0 ;
risk1 = 100 ;
threshold2 = 35;
risk2 = 10^5 ;

% compute the function for all the frames
for i=2:nbFrame-1
    % compute a distance image from the paper
    thoseFrame = matrixImage2(:,:,i-1:i+1) ;
    distFrame = computeDistFrame(thoseFrame) ;
    
    % apply 2 threshold to this distance map
    mask1 = distFrame>threshold1 ;
    mask2 = distFrame>threshold2 ;
    
    % reduce the noise and false alarm
    connectCom1 = reduceNoise(mask1, distFrame, risk1) ;
    connectCom2 = reduceNoise(mask2, distFrame, risk2) ;
    
    % apply an hysteresis threshold to our noise free masks
    mask = double(hysthresh(connectCom1,connectCom2)) ;
    
    % creating the new video
    newMask = mask(2:end-1, 2:end-1) ;
    thoseFrame = double(thoseFrame(2:end-1, 2:end-1,:)) ;
    
    % remove object which moves 
    %newMask = maskWithoutMove(thoseFrame, mask) ;
    
    oldFrame = thoseFrame(:,:,2) ;
    thoseFrame(:,:,2) = [] ;
    
    % fill the hools with the means of the surounding frames
    newFrameInter = mean(repmat(newMask, [1,1,2]).*thoseFrame,3) ;
    newFrame = newFrameInter.*newMask + oldFrame.*(1-newMask) ;
    
    resultMatrix(:,:,i) = newFrame ;
end
resultMatrix = uint8(resultMatrix) ;
end

% reduce the noise and false alarm
function result = reduceNoise(mask, frame, risk)
% create a map of objects from a map of pixels
se = [0 1 0 ; 1 1 1 ; 0 1 0] ;
mask = imdilate(imerode(mask,se),se) ;
conectCom = bwconncomp(mask) ;

% number of detected blotched
numberBloch = conectCom.NumObjects ;

for j=1:numberBloch
    % compute the probability for this blotched
    listPixel = conectCom.PixelIdxList{j} ;
    probaT = computProbTotal(frame,listPixel ) ;
    
    % if the total probability of the blotch is above a risk, we dont keep
    % it
    if probaT > risk
        conectCom(1,j) = [] ;
    end
end
result = conectCom ;
end

% compute a distance image 
function distFrame = computeDistFrame(frames)
[M,N,~] = size(frames) ;
distFrame = zeros(M,N) ;

for j=2:M-1
    for k=2:N-1
        intVal = frames(j,k,2) ;
        
        % value of the distance around the concerned pixel
        pixelsVals = zeros(6,1) ;
        pixelsVals(1:3,1) = frames(j-1:j+1,k,1) ;
        pixelsVals(4:6,1) = frames(j-1:j+1,k,3) ;
        
        % compute the distance for this pixel
        dist = computeDist(pixelsVals, intVal) ;
        distFrame(j,k) = dist ;
    end
end
end

% compute a distance according to the paper
function dist = computeDist(pixelsValues, currentInt)
if min(pixelsValues(:)) >= currentInt
    dist = min(pixelsValues(:))-currentInt ;
elseif currentInt >= max(pixelsValues(:))
    dist = currentInt-max(pixelsValues(:)) ;
else
    dist =0;
end
end

% compute gaussian proba for a value with a given mean and cov
function like = calcGaussianProb(data,gaussMean,gaussCov)
like = (1/(((2*pi))*det(gaussCov)^0.5))*exp(-0.5*(data-gaussMean)'*(gaussCov^(-1))*(data-gaussMean))  ;
end

% compute the probability for this blotched
function probaTot = computProbTotal(distFrame,listPixel)
numPix = size(listPixel,1) ;
distBlo = zeros(numPix,1) ;

% list of distances for this pixel
for k=1:numPix
    [indI, indJ] = ind2sub(size(distFrame),listPixel(k)) ;
    dist = distFrame(indI,indJ) ;
    distBlo(k,1) = dist ;
end

% computer the mean for this blotch
meanDi = mean(distBlo) ;
proba = zeros(numPix,1) ;

for k=1:numPix
    % compute proba with the mean of this bloched and a variance of 9
    proba(k,1) = calcGaussianProb(distBlo(k,1),meanDi,9) ;
end
% return the normalized total proba
probaTot = sum(proba,1)/numPix ;
end

function mask = compuMask(CC)
numbBloc = CC.NumObjects ;
mask = zeros(CC.ImageSize) ;

for i=1:numbBloc
    pixBloch = CC.PixelIdxList{i} ;
    numPix = size(pixBloch,1) ;
    for j=1:numPix
        [indI, indJ] = ind2sub(CC.ImageSize,pixBloch(j)) ;
        mask(indI, indJ) = 1 ;
    end
end
end

% return the hysteresis threshold between the two
function mask = hysthresh(comp1,comp2)
% low threshold
mask1 = compuMask(comp1) ;
% high thresohold
mask2 = compuMask(comp2) ;
[mask2X, mask2Y] = find(mask2 > 0);

mask = bwselect(mask1,mask2Y, mask2X, 8);
end


% remove objects which move 

% function result = maskWithoutMove(frames, mask)
% % compute the object 
% conectCom = bwconncomp(mask) ;
% 
% % compute the optical flow  
% opticFlow = opticalFlowLK ;
% frames(:,:,1) ;
% flow1 = estimateFlow(opticFlow,frames(:,:,2)) ;
% frames(:,:,3) ;
% flow2 = estimateFlow(opticFlow,frames(:,:,2)) ;
% flow = cat(3,flow1.Vx,flow1.Vy,-flow2.Vx,-flow2.Vy) ;
% 
% % compute the mean in every directions
% meanFlow = mean(flow,3) ;
% 
% if conectCom.NumObjects == 0
%     result = mask ;
% else
%     for i=1:conectCom.NumObjects
%         listPixel = conectCom.PixelIdxList{i} ;
%         [pixI, pixJ] = ind2sub(size(mask),listPixel) ;
%         pixI = uint8(pixI) ;
%         pixJ = uint8(pixJ) ;
% 
%         meanTest=0 ;
%         for j=1:size(pixI)
%            for k=1: size(pixJ)
%                 meanTest = meanTest + meanFlow(pixI(j,1), pixJ(k,1)) ;
%            end
%         end
%         meanTest = meanTest/size(pixI,1) ;
%         if meanTest > 200
%             conectCom.PixelIdxList{i}  = [] ;
%         end  
%     end
%     result = compuMask(conectCom) ;
% end
% end

