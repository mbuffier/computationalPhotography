function output = labs3(path, prefix, first, last, digits, suffix)

% Read a sequence of images and correct the film defects. This is the file 
% you have to fill for the coursework. Do not change the function 
% declaration, keep this skeleton. You are advised to create subfunctions.
% 
% Arguments:
%
% path: path of the files
% prefix: prefix of the filename
% first: first frame
% last: last frame
% digits: number of digits of the frame number
% suffix: suffix of the filename
%
% This should generate corrected images named [path]/corrected_[prefix][number].png
%
% Example:
%
% mov = labs3('../images','myimage', 0, 10, 4, 'png')
%   -> that will load and correct images from '../images/myimage0000.png' to '../images/myimage0010.png'
%   -> and export '../images/corrected_myimage0000.png' to '../images/corrected_myimage0010.png'
%

% Your code here

tic 
% I load all  the images in a matrix 
matrixImage = load_sequence(path, prefix, first, last, digits, suffix) ;
toc 
% last 2.348s
tic
% detect the scene cuts 
[frameChange, ~] = detectSceneCut(matrixImage) ;
toc
% last 13.726s
tic
% reduce the flicker effects
resultMatrix1 = reduceFlicker(matrixImage, frameChange) ;
toc 
% last 13.726s
tic
% correct the blotches 
resultMatrix2 = blotchCorrec(resultMatrix1) ;
toc
% last 12min
tic
% correct the vertical effect in the last sequence
matrixImageLast = resultMatrix2(:,:,frameChange(2,1)+1:end) ;
resultMatrixLastSeq = verticalArtefact(matrixImageLast) ;
resultMatrix2(:,:,frameChange(2,1)+1:end) = resultMatrixLastSeq ;
toc
% last 2min30
tic
% reduce the shake
resultMatrixFinal = reduceShake(resultMatrix2,frameChange) ;
toc
% last 5min20

% save the result frame by frame
save_sequence(resultMatrixFinal, 'resultImage', 'result_', 1, 3) ;

% create the final video 
createVideo(uint8(resultMatrixFinal), 'finalVideo') ;
end

% export the final video
function r =  createVideo(matrix, name)
% matrix: matrix of images to export
% path: path of the files
% name : name of the filename

numFrame = size(matrix,3) ;
% initialization of the video 
filename = ['resultVideo/' name '.avi'];
v = VideoWriter(filename,'Uncompressed AVI');
open(v) ;

for i=1:numFrame
    writeVideo(v,matrix(:,:,i)) ;
end

end
