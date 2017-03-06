function result = reduceFlicker(matrixImage, frameChange)

%extract the number of frame and the matrixImage for each sequences
[M,N, numTotalFrame] = size(matrixImage) ;
matrixImage = double(matrixImage);

% size of the result
result = zeros(M, N, numTotalFrame) ;

% in case we only take part of the video
if frameChange == 0
    frameChange = [0 ; numTotalFrame] ;
    numSeq = 1 ;
else
    frameChange = [0; frameChange; numTotalFrame] ;
    numSeq = size(frameChange,1)-1 ;
end

% to treat sequences separately
for i=1:numSeq
    % extract the images of the sequence
    imageSeq = matrixImage(:,:,frameChange(i)+1:frameChange(i+1)) ;

    % apply the flicker reduce effect
    imageSeqCorrect = redflicker(imageSeq) ;
    
    result(:,:,frameChange(i)+1:frameChange(i+1)) = imageSeqCorrect ;
end

result = uint8(result)  ;

end

% reduce flicker in 1 sequence by histogram matching
function result = redflicker(matrixImage)

[M, N, numFrame] = size(matrixImage) ;
result = zeros(M, N, numFrame) ;

% we compute the mean and the variance of the sequence
means = reshape(mean(mean(matrixImage,1),2), [1 numFrame]) ;
meanTot = mean(means) ;
meanTotArray = repmat(meanTot, [1 numFrame]) ;
var = ((means-meanTotArray)*(means-meanTotArray).')/numFrame ;
std = sqrt(var) ;

% we compute the frame out of the mean
mask1 = means>meanTot+std ;
mask2 = means<meanTot-std ;
maskT = mask1+mask2 ;

add = 5 ;
halfadd = floor(add/2) ;
matrixImageNew = padarray(matrixImage, [0 0 halfadd], 'replicate', 'both') ;

for i=1+halfadd:numFrame+halfadd
    % if the mean of the frame isn't in the average we correct it  using
    % histogram matching on 2 frames before and 2 frames after
    if maskT(i-halfadd) == 1
        patch = cat(3, matrixImageNew(:,:,i-halfadd:i-1),matrixImageNew(:,:,i+1:i+halfadd)) ;
        
        % create the histogram target with the good frames
        histoTarget = computeHistos(patch) ;
        cumulHisTarget = round(cumsum(histoTarget)) ;
        
        % create the histograme for the frame
        h = histogram(matrixImageNew(:,:,i-halfadd)) ;
        h.NumBins = 256 ;
        h = h.Values ;
        
        % compute the cumulative histogram
        cumulHis = cumsum(h) ;
        
        % compute the map function
        mapFunction = computeMapFonc(cumulHis, cumulHisTarget) ;
        
        % apply the map function
        result(:,:,i-halfadd) = mapHisto(mapFunction, matrixImageNew(:,:,i-halfadd));
    else
        % else we don't correct the frame
        result(:,:,i-halfadd) = matrixImageNew(:,:,i-halfadd);
    end
end


end

% compute the histogram mean for a sequence of images
function hisTarget = computeHistos(matrixImages)
[~,~,numFrame] = size(matrixImages) ;
histos = zeros(numFrame,256) ;

for i=1: numFrame
    his = histogram(matrixImages(:,:,i)) ;
    his.NumBins = 256 ;
    histos(i,:) = his.Values ;
end
hisTarget = mean(histos, 1) ;
end

% given 2 cumulative histograms, compute the mapping function
function mapFunction = computeMapFonc(cumHistImg, cumHistTarget)
mapFunction = zeros(1, 255) ;
for i=1:255
    for k=1:255
        newValue = 0 ;
        if (cumHistTarget(k) >=cumHistImg(i))
            newValue = k ;
            break;
        end
    end
    mapFunction(i) = newValue ;
end
end

% given a mapping function and a frame, give the resulting frame
function resultFrame = mapHisto(mapfunc, frame)
[M,N] = size(frame) ;
resultFrame = zeros(M, N) ;
for i=1:M
    for j=1:N
        % change the pixel value accordingly to the mapping
        resultFrame(i,j) = mapfunc(frame(i,j)+1) ;
    end
end
end


