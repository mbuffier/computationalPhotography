function result = projectorCalibration(folderName, imagesName, numberImage)

% name of images and right folder 

% for the intrisic of the camera, use of the librairy for calibration over
% cam_calib
% for the intrisic of the camera, use of the librairy for calibration over
% proj_calib (created using imProj)

% compute the position for each point according to what's given for the check in the projector :

% our data : 
% startCheck = [440, 200] ;
% sizeCheck = 400 ;

% given data : 
startCheck = [518, 120] ;
sizeCheck = 299 ;

posCheck = [startCheck(1), startCheck(1)+sizeCheck,startCheck(1)+sizeCheck, startCheck(1);
    startCheck(2), startCheck(2), startCheck(2)+sizeCheck,startCheck(2)+sizeCheck];


for i=1:numberImage
    % selection of the corner in the projected chequerboard seen by the
    % camera
    imProject = imread([folderName '/' imagesName num2str(i) '.jpg']) ;
    fig1 = figure, imshow(imProject) ;
    
    [imProjectJ, imProjectI] = getline(fig1) ;
    ptsProj = [imProjectJ' ;imProjectI'] ;
    
    % compute the best homography between the image in the chequerboard and
    % the projected image seen by the camera
    bestHom = calcBestHomography(ptsProj, posCheck) ;
    bestHom = maketform('projective',bestHom') ;
    
    % define the size of the projector
    Xdata = [1, 1024] ;
    Ydata = [1, 768] ;
    
    % transform the image seen by the camera into the projector frame
    resultImage =  imtransform(imProject,bestHom, 'XData', Xdata, 'YData', Ydata) ;
    
    % write the result
    imwrite(resultImage, [folderName '/proj_calib' num2str(i) '.jpg']) ;
end

end

function H = calcBestHomography(pts1Cart, pts2Cart)

M = size(pts1Cart,2) ;

A = zeros(2*M,9);
index = 1 ;

for i = 1:M
    A(index,:) = [0, 0,0, -pts1Cart(1,i), -pts1Cart(2,i), -1, pts2Cart(2,i).*pts1Cart(1,i), pts2Cart(2,i).*pts1Cart(2,i), pts2Cart(2,i)] ;
    A(index+1,:) = [pts1Cart(1,i) pts1Cart(2,i) 1 0 0 0 -pts2Cart(1,i)*pts1Cart(1,i) -pts2Cart(1,i)*pts1Cart(2,i) -pts2Cart(1,i)] ;
    index = index+2 ;
end

h = solveAXEqualsZero(A);

H = reshape(h, [3,3]) ;
H = H.' ;
end

function x = solveAXEqualsZero(A)
[~,~,V] = svd(A) ;
N = size(V,2) ;
x = V(:,N) ;
end
