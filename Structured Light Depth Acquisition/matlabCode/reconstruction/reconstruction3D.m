function result = reconstruction3D(folderName, imageName,first,digit, format,typeCalib)

% foldername is the name of the folder which contains the object we want to
% reconstruct

% typeCalib has 3 choices : 
% - 1 == provided calibration
% - 2 == calibration found for the synthetic dataset
% - 3 == calibration for the real dataset provided
% - 4 == calibration for my own real dataset

%************ Step 1 : load the  uv code for the projector to test the function  **********
% if ~exist('uvProj.mat')
%     trueCodedPixels = findPixelsCode('Gray_code_pattern', 'gray', false) ;
%     disp('uv code for the projector computed') ;
%     save('uvProj.mat', 'trueCodedPixels') ;
% else
%     load('uvProj.mat') ;
%     disp('uv code for the projector computed') ;
% end

%************ Step 2 : load the  uv code of the object seen from the camera  **********

text = ['../uvCode/uv' folderName '.mat'] ;
if ~exist(text)
    imageCodedPixels = findPixelsCode(['../data_sets/' folderName], imageName, first,digit, format , true) ;
    disp('uv code for the image computed') ;
    save(text, 'imageCodedPixels') ;
else
    load(text) ;
    disp('uv code for the image computed') ;
end

% blur the result to avoid multiple pixels with the same code :
[MIm,NIm, ~] = size(imageCodedPixels) ;

%************ Step 2.5 : Blur in the case of real data  **********

% For synthetic data :  
%matrixCode = imageCodedPixels ;

%for real data : 
matrixCode =zeros(MIm,NIm,2) ;

for i =1:max(MIm,NIm)
    if i < min(MIm,NIm)
        % first part incodes rows in my code -> smooth vertically  
        matrixCode(:,i,1) = medfilt1(imageCodedPixels(:,i,1),5) ;
        % second part incodes colums in my code -> smooth horizontally   
        matrixCode(i,:,2) = medfilt1(imageCodedPixels(i,:,2),5) ;
    else
        matrixCode(:,i,1) = medfilt1(imageCodedPixels(:,i,1),5) ;
    end
end

%************ Step 3 : load the intrinsic and extrinsic matrices  **********
% load the good matrices 
if typeCalib == 1
    givenCalib ;
elseif typeCalib == 2
    foundCalib ;
elseif typeCalib == 3
    real_given_calib ;
elseif typeCalib == 4
    ownData_calib ;
else 
    error('The calibration you ask for is not provided, please choose between 1, 2, 3 or 4') ;
end

disp('Matrices for the projector and camera computed') ;

%************ Step 4 : Move the camera to generate another images  **********
%we create a new matrix to project the 3D point into the new rotated
%position
camExtMoved = zeros(3,4)  ;

% I compute a rotation on the z axis to observe a differente depth map 
rot = [cosd(40), -sind(40), 0;
    sind(40), cosd(40), 0 ;
    0 , 0, 1,] ;

camExtMoved(1:3,1:3) =  CameraExt(1:3,1:3)*rot;
camExtMoved(1:3,4) =  CameraExt(1:3,4) ;

%************ Step 5 : Compute the depth in both images   **********

% create the futur resulting depth maps
depthImage = zeros(MIm,NIm) ;
depthImageMovedCamera = zeros(MIm,NIm) ;
pointCloud = zeros(MIm*NIm,3) ;
index=0 ;

% for each pixel, compute the unique depth 
for i=1:MIm
    for j=1:NIm
        current_uv_code = reshape(matrixCode(i,j,:), [2,1]) ;
        code_test = imageCodedPixels(i,j,:) ;
        
        % if sum(code_test(:)) <= 2 , the pixel is always black so in the
        % background or you are unsure about it 
        if sum(code_test(:)) > 2
            
            % coordinates of the images in homogeneous coordinates
            pImg = [j;i;1] ;
            pProj = [current_uv_code(2);current_uv_code(1);1];
            
            % in camera and projector frame
            pImgp = CameraIn\pImg;
            pProjp = ProjectIn\pProj;
            
            % we'll solve the linear system for both the camera positions
            A = zeros(4,3);
            b = zeros(4,1);
            
            %first camera in front of the object :
            A(1,:) = [CameraExt(3,1)*pImgp(1)-CameraExt(1,1), CameraExt(3,2)*pImgp(1)-CameraExt(1,2), CameraExt(3,3)*pImgp(1)-CameraExt(1,3)];
            A(2,:) = [CameraExt(3,1)*pImgp(2)-CameraExt(2,1), CameraExt(3,2)*pImgp(2)-CameraExt(2,2), CameraExt(3,3)*pImgp(2)-CameraExt(2,3)];
            b(1:2) = [CameraExt(1,4)-CameraExt(3,4)*pImgp(1); CameraExt(2,4)-CameraExt(3,4)*pImgp(2)];
            
            % projector : doesn't change for both poses
            A(3,:) = [ProjectExt(3,1)*pProjp(1)-ProjectExt(1,1), ProjectExt(3,2)*pProjp(1)-ProjectExt(1,2), ProjectExt(3,3)*pProjp(1)-ProjectExt(1,3)];
            A(4,:) = [ProjectExt(3,1)*pProjp(2)-ProjectExt(2,1), ProjectExt(3,2)*pProjp(2)-ProjectExt(2,2), ProjectExt(3,3)*pProjp(2)-ProjectExt(2,3)];
            b(3:4) = [ProjectExt(1,4)-ProjectExt(3,4)*pProjp(1); ProjectExt(2,4)-ProjectExt(3,4)*pProjp(2)];
            
            %compute the least squares solution of our system 
            % w is in the world coordinates 
            w1 = A\b;

            w1_result = CameraExt*[w1;1] ;
            
            % we reproject using the computed camera to have the other view
            % of the depth 
            w2_result = camExtMoved*[w1;1] ;

            %store the depth value in the images
            depthImage(i,j) = w1_result(3) ;
            depthImageMovedCamera(i,j) = w2_result(3) ;
            
            index = index + 1 ;
            pointCloud(index,:) = w1 ;
        else
            % we put it in the background 
            depthImage(i,j) = -1 ;
            depthImageMovedCamera(i,j) = -1 ;
        end
    end
end
pointCloud = pointCloud(1:index,:) ;

% ********** Step 6 : Store and prit the result for depth and the 3D points meshes ********
mask = depthImage == -1 ;
% try to print a depth image were we can see the changes
depthImage = (1-mask).*depthImage + mask.*(max(depthImage(:)+0.01)) ;

mask2 = depthImageMovedCamera == -1 ;
depthImageMovedCamera = (1-mask2).*depthImageMovedCamera + mask2.*(max(depthImageMovedCamera(:)+0.0001)) ;

% print the depth maps 
fig1 = figure ;
set(fig1, 'Position', [0 0 1500 400]) ;
subplot(1,2,1) , imagesc(depthImage), title('Depth image')
subplot(1,2,2) , imagesc(depthImageMovedCamera), title('Depth image for the moved camera : angle of 40?')

saveas(gcf, ['resultFoundCalib' folderName '.jpg']) ;

% store the pointCloud 
namePly = ['pointCloud_' folderName] ;
createPLY(pointCloud, namePly) ;
end