function result = verticalArtefact(matrixImage)

% size of the frames
[M,N,numberframe] = size(matrixImage) ;
correction = zeros(M,N,numberframe) ;

% values chosen after testing 
order = 12 ;
threshold = 70 ;
windowSize = 5 ;

% go throught all the frames of the sequence to erase artefact
for i=1:numberframe
    thisFrame = double(matrixImage(:,:,i)) ;    
    
    newFrame = correct(thisFrame, order, threshold, windowSize) ;

    correction(:,:,i) = newFrame ;
end

result = correction ;
end

% apply a median filter to the frame 
function result = filterFrame(frame, order)
[M,N] = size(frame) ;
result = zeros(M,N) ;

% apply the correction for each row
for j=1:M
    thisRow = frame(j,:) ;
    newRow = medfilt1(thisRow,order) ;
    result(j,:) = newRow ;
end
% resulting newFrame
result = uint8(result) ;
end

% correct the flat part of the image while keeping the hight gradient 
function result = correct(img, order, ind, windowS)
% compute the gradient  to detect details areas
[Gmag,~] = imgradient(img) ;
[M,N] = size(img) ;
result = zeros(M,N) ;

% filter the frame 
newFrame = filterFrame(img, order);
Gmag = padarray(Gmag, [windowS,windowS]) ; % avoid matlab errors 
halfS = floor(windowS/2) ;

for i=1+halfS:M-halfS
    for j=1+halfS:N-halfS
        % compute a windowS*windowS patch around the pixel to find it's
        % mean value 
        testPatch = Gmag(i-halfS:i+halfS,j-halfS:j+halfS) ;
        meanTest = mean(testPatch(:)) ;
        % if the mean is above the threshold, we keep the unfilter image :
        % we are in a high gradient zone 
       if meanTest > ind
           result(i-halfS,j-halfS) = img(i-halfS,j-halfS) ;
       else % else we take the filtered image
           result(i-halfS,j-halfS) = newFrame(i-halfS,j-halfS) ;
       end
    end
end
result =  uint8(result) ;
end

% to test my results and find the best values 
function r = test2(thisFrame, order)
test1 = correct(thisFrame, order, 100, 3) ;
test2 = correct(thisFrame, order, 100, 5) ;
test3 = correct(thisFrame, order, 100, 7) ;


test4 = correct(thisFrame, order, 70, 3) ;
test5 = correct(thisFrame, order, 70, 5) ;
test6 = correct(thisFrame, order, 70, 7) ;


test7 = correct(thisFrame, order, 150, 3) ;
test8 = correct(thisFrame, order, 150, 5) ;
test9 = correct(thisFrame, order, 150, 7) ;

figure, 
subplot(3,3,1), imshow(test1) ;
subplot(3,3,2), imshow(test2) ;
subplot(3,3,3), imshow(test3) ;
subplot(3,3,4), imshow(test4) ;
subplot(3,3,5), imshow(test5) ;
subplot(3,3,6), imshow(test6) ;
subplot(3,3,7), imshow(test7) ;
subplot(3,3,8), imshow(test8) ;
subplot(3,3,9), imshow(test9) ;

end
