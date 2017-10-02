function r = mainBlending(jsonFileName, imageFolder, numberOfFrame)

% jsonFileName is the name of the json file to convert 
% imageFolder the folder containing the images 
% numberOfFrame the number of frame in the result 


% get back the size of the images 
imageName = ['SfM/' imageFolder '/test00.jpg'] ;
firstImage = imread(imageName) ;
[M, N, ~] = size(firstImage) ;

% convert the JSON file
[point_poses, cameras_info, point_seen_byC, camera_intrinsic] = conversion(jsonFileName) ;

% find the new path for the center and for the rotation  
camera_center = computeCenterRot(cameras_info) ;
newCurveCenter = spline3dCurveInterpolation(camera_center(:,2:4), numberOfFrame) ;
newCurveQuat = spline4dCurveInterpolation(camera_center(:,5:8), numberOfFrame) ;

% create the kd-tree with the center positions 
tree = createns(camera_center(:,2:4),'Distance','euclidean') ;

matrixResult = zeros(M, N, 3, numberOfFrame, 'uint8') ;

for i = 1:numberOfFrame
    % find the closest camera
    [Idx, dist] = knnsearch(tree,newCurveCenter(i,:), 'K', 2);
    [coef1, coef2] = findCoef(dist(1), dist(2)) ;
    
    % compute the extrinsic for the virtual camera
    thisExtrinsic = computeExtrinsic(newCurveCenter(i,:), newCurveQuat(i,:));
    
    if coef1 > 0.75
        D_field = zeros(M, N,2) ;
        
        % ************ for the closest camera *************
        thisViewKey = camera_center(Idx(1), 1) ;
        % find the point to compute the flows
        [pointNewPose, pointOldPose] = findPoints(thisExtrinsic, point_seen_byC{1,thisViewKey},camera_intrinsic, point_poses, M, N) ;
        
        % Find the displacement field from the movements
        [Xq,Yq] = meshgrid(1:N, 1:M);
        
        % the value of the optical flow for the feature points
        value_X = pointOldPose(1,:)-pointNewPose(1,:) ;
        value_Y = pointOldPose(2,:)-pointNewPose(2,:) ;
        
        % interpolation of the field
        FX = scatteredInterpolant(pointOldPose',value_X');
        FY = scatteredInterpolant(pointOldPose',value_Y');
        D_field(:,:,1) = imgaussfilt(FX(Xq, Yq),10) ;
        D_field(:,:,2) = imgaussfilt(FY(Xq, Yq), 10) ;
        
        % find the image associate to this camera pose
        thisImageNumber = camera_center(Idx(1),1) - 1 ;
        imageName = ['SfM/' imageFolder '/test0' num2str(thisImageNumber) '.jpg'] ;
        thisImage = imread(imageName) ;
        
        % fill in the result
        newImage = imwarp(thisImage,D_field);
        matrixResult(:,:,:,i) = newImage ;
    else
        D_field_close = zeros(M, N,2) ;
        D_field_far = zeros(M, N,2) ;
        
        % ************ for the closest camera *************
        thisViewKey1 = camera_center(Idx(1), 1) ;
        % find the point to compute the flows
        [pointNewPose, pointOldPose] = findPoints(thisExtrinsic, point_seen_byC{1,thisViewKey1},camera_intrinsic, point_poses, M, N) ;
        
        % Find the displacement field from the movements
        [Xq,Yq] = meshgrid(1:N, 1:M);
        
        % the value of the optical flow for the feature points
        value_X = pointOldPose(1,:)-pointNewPose(1,:) ;
        value_Y = pointOldPose(2,:)-pointNewPose(2,:) ;
        
        % interpolation of the field
        FX = scatteredInterpolant(pointOldPose',value_X');
        FY = scatteredInterpolant(pointOldPose',value_Y');
        D_field_close(:,:,1) = imgaussfilt(FX(Xq, Yq),10) ;
        D_field_close(:,:,2) = imgaussfilt(FY(Xq, Yq), 10) ;
        
        % ************ For the 2de closest camera *************
        thisViewKey2 = camera_center(Idx(2), 1) ;
        % find the point to compute the flows
        [pointNewPose, pointOldPose] = findPoints(thisExtrinsic, point_seen_byC{1,thisViewKey2},camera_intrinsic, point_poses, M, N) ;
        
        % the value of the optical flow for the feature points
        value_X = pointOldPose(1,:)-pointNewPose(1,:) ;
        value_Y = pointOldPose(2,:)-pointNewPose(2,:) ;
        
        % interpolation of the field
        FX = scatteredInterpolant(pointOldPose',value_X');
        FY = scatteredInterpolant(pointOldPose',value_Y');
        D_field_far(:,:,1) = imgaussfilt(FX(Xq, Yq),10) ;
        D_field_far(:,:,2) = imgaussfilt(FY(Xq, Yq), 10) ;
        
        % ********** find the warp images for both cameras *******
        % for the first one
        thisImageNumber1 = camera_center(Idx(1),1) - 1 ;
        imageName1 = ['SfM/' imageFolder '/test0' num2str(thisImageNumber1) '.jpg'] ;
        thisImage1 = imread(imageName1) ;
        newImage1 = imwarp(thisImage1,D_field_close);
        
        % for the second one
        thisImageNumber2 = camera_center(Idx(2),1) - 1 ;
        imageName2 = ['SfM/' imageFolder '/test0' num2str(thisImageNumber2) '.jpg'] ;
        thisImage2 = imread(imageName2) ;
        newImage2 = imwarp(thisImage2,D_field_far);
        
        % ********* compute the coef to blend the images *********
        [alpha2, alpha1] = findCoef(coef1, coef2) ;

        newImage = blend(newImage1, newImage2, alpha1, alpha2) ;
        
        matrixResult(:,:,:,i) = newImage ;
    end
    
    
end

createVideo(matrixResult, 'test') ;

end

% compute the rotation and center from the camera info converted from the
% JSON file 
function camera_center_rot  = computeCenterRot(cameras_info)
% camera_center_rot contains the id of the camera, its center and its
% rotation in quaternion 
number_views = size(cameras_info,2) ;
camera_center_rot = zeros(8,number_views) ;

index = 0 ;
for i=1:number_views
    % some camera info are empty => not used in the SfM algorithm 
    if  ~isempty(cameras_info{1,i})
        index = index+1 ;
        % center position
        camera_center_rot(1,index) = i ;
        camera_center_rot(2:4,index) = cameras_info{1,i} ;
        
        % rotation 
        thisRot = cameras_info{2,i} ;
        thisQuat = qGetQ(thisRot) ;
        camera_center_rot(5:8, index) = thisQuat ; 
    end
end
camera_center_rot = camera_center_rot(:,1:index) ;
camera_center_rot = camera_center_rot' ;

% reordered the points with respect to the first camera position 
from_start = camera_center_rot(:,2:4) - repmat(camera_center_rot(1,2:4), [index,1]) ; 
dist_from_Start = from_start(:,1).^2+from_start(:,2).^2+from_start(:,3).^2 ;
camera_center_rot = sortrows([dist_from_Start, camera_center_rot]) ;

% extract the result
camera_center_rot = camera_center_rot(:,2:end) ;
end

% center is the camera position but not the translation with respect to the
% world frame so we must find the translation to obtain the extrinsic
% matrix
function thisExtrinsic = computeExtrinsic(center, quater)
thisRot = qGetR( quater ) ;
thisTrans = -thisRot*center' ;

thisExtrinsic = [thisRot, thisTrans] ;
end

% find the point seens by this camera and the position in the virtual image
% frame 
function [pointNewPose, pointOldPose] = findPoints(newExtrinsic, point_seen_byC,camera_intrinsic, point_poses, M, N)

numberPoints = size(point_seen_byC,2) ;
index = 1 ;
pointOldPose = zeros(2,numberPoints) ;
pointNewPose = zeros(2,numberPoints) ;

% for each 3D point, we reprojecte them in the virtual camera frame to see
% their new position 
for i=1:numberPoints
    % get back the 3D coordinates of this point 
    thisPointIndex = point_seen_byC(1,i) ;
    homCoord = [point_poses(:,thisPointIndex) ; 1] ;
    
    % find the 2D position in image frame 
    newPose = camera_intrinsic*newExtrinsic*homCoord ;
    newPose = newPose(1:2,1)./newPose(3,1) ;
    
    % if the point is in the image -> we keep it 
    if (newPose(1,1) < N && newPose(2,1) < M && newPose(1,1) > 0 && newPose(2,1) > 0 )
        pointOldPose(:,index) = point_seen_byC(2:3,i) ;
        pointNewPose(:,index) = newPose ;
        index = index+1 ;
    end
end

% result 
pointNewPose = pointNewPose(:,1:index-1) ;
pointOldPose = pointOldPose(:,1:index-1) ;
end


% compute the coefficient for the flow in function of the distance
function [coef1, coef2] = findCoef(dist1, dist2)
scale =  dist2/ dist1 ;
coef1 = scale/(1+scale) ;
coef2 = 1-coef1 ;
end


function newImage = blend(image1, image2, alpha1, alpha2)
% simple blending function for now 
newImage = image1.*alpha1 + image2.*alpha2 ;
end

% export the final video
function r =  createVideo(matrix, name)
% matrix: matrix of images to export
% path: path of the files
% name : name of the filename

numFrame = size(matrix,4) ;
% initialization of the video
filename = ['videos/result/' name '.avi'];
v = VideoWriter(filename,'Uncompressed AVI');
open(v) ;

for i=1:numFrame
    writeVideo(v,matrix(:,:,:,i)) ;
end

end
