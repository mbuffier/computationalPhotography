function numberFrame = fromVideoToImages(fileName, modulo, folderName)

% fileName is the name of the video in the videos folder
% modulo determines the number of frame to keep 

FileName = ['videos/' fileName] ;
thisMovie = VideoReader(FileName);
index = 0 ;
numFrameTotal = 0 ;

% read all the frames 
while hasFrame(thisMovie)
    thisFrame = readFrame(thisMovie);
    
    % only keep the number in the modulo
    if (~mod(numFrameTotal, modulo))
        
        %thisFrame = rgb2gray(imresize(thisFrame, 0.3)) ;
        thisFrame = imresize(thisFrame, 0.3) ;
        
        nameFrame = ['SfM/' folderName '/test0' num2str(index) '.jpg'] ;
        index = index + 1 ;
        imwrite(thisFrame, nameFrame) ;
    end
    numFrameTotal =numFrameTotal+1 ;
end

numberFrame = index ;
end

