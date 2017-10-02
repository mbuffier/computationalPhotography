function result = projectorCalibration

% for the intrisic of the camera, use of the librairy for calibration over
% cam_calib
% for the intrisic of the camera, use of the librairy for calibration over
% proj_calib (created using imProj)

% for the extrinsic between the camera and projector, use one pair of
% images CB_proj_frame and imProj1

% compute the position for each point according to what's given for the check in the projector :
startCheck = [378, 277] ;
sizeSquare = 45 ;
sizeCheck = 270 ;
posCheck = [startCheck(1)+sizeSquare, startCheck(1)+sizeCheck-sizeSquare,startCheck(1)+sizeCheck-sizeSquare, startCheck(1)+sizeSquare;
    startCheck(2)+sizeSquare, startCheck(2)+sizeSquare, startCheck(2)+sizeCheck-sizeSquare,startCheck(2)-sizeSquare+sizeCheck];


for i=1:3
    % selection of the corner in the projected chequerboard seen by the
    % camera
    imProject = imread(['imProj' num2str(i) '.jpg']) ;
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
    im2trans = imread(['cam_calib' num2str(i) '.jpg']) ;
    result =  imtransform(im2trans,bestHom, 'XData', Xdata, 'YData', Ydata) ;
    
    % write the result
    imwrite(result, ['proj_calib' num2str(i) '.jpg']) ;
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
