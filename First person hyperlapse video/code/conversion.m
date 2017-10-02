function [point_poses, cameras_info, point_seen_byC, camera_intrinsic] = conversion(filename)

% point_poses contains the poses of the landmarks 

% cameras_info is contains the center of the camera and
% the rotation in the indice of the frame

% point_seen_byC contains for each point the id of the cameras which see it
% with its position in the image frame 

% camera_intrinsic contains a matrix with the intrinsic parameters


% *************** Step 1 : open and red the file *************
fid = fopen(filename);
raw = fread(fid,inf);
str = char(raw');
fclose(fid);

data = JSON.parse(str) ;

%*********  Step 2 : Construction of the camera_intrinsic matrix *********
focal_length = data.intrinsics{1,1}.value.ptr_wrapper.data.focal_length ;
c_u = data.intrinsics{1,1}.value.ptr_wrapper.data.principal_point{1,1} ;
c_v = data.intrinsics{1,1}.value.ptr_wrapper.data.principal_point{1,2} ;

camera_intrinsic = [focal_length, 0, c_u ;
                    0 , focal_length, c_v;
                    0 , 0, 1] ;

%********* Step 4 : Correspondance between views and poses *********

% will contain the correspondance between the id of the camera in the openMVG result and
%the frame number 

% views = number of frame given to OpenMVG
number_views = size(data.views,2) ;
correspondances = zeros(1,number_views) ;
for i=1:number_views
    camera_name = data.views{1,i}.value.ptr_wrapper.data.filename ;
    index_frame = findIndex(camera_name) ;
    key = data.views{1,i}.key+1;
    correspondances(1,key) = index_frame ;
end

%********* Step 4 : Construction of the camera_poses vector *********
% poses = number of found camera extrinsic parameters  
number_poses = size(data.extrinsics,2) ; 
% output with the centers and the rotations
cameras_info = cell(2,number_views) ;

% for each poses 
for i=1: number_poses
    % initialization of rotation and center
    thisRot = zeros(3,3) ;
    camera_center = zeros(3,1) ;
    
    % find the frame number
    thisKey = data.extrinsics{1,i}.key + 1;
    this_indexFrame =  correspondances(1,thisKey) ;
    
    % fill the center and the rotation for this poses 
    for j=1:3
        camera_center(j,1) = data.extrinsics{1,i}.value.center{1,j} ;
        for k=1:3
           thisRot(j,k) = data.extrinsics{1,i}.value.rotation{1,j}{1,k} ;
        end
    end
    % file the camera info vector at the right position 
    cameras_info{1,this_indexFrame} = camera_center ;
    cameras_info{2,this_indexFrame} = thisRot ;
end

%********* Step 5 : Construction of the point_seen_byC and point_poses vector *********
% number of feature points in the reconstruction 
number_points = size(data.structure,2) ;
point_seen_byC = cell(1, number_views) ;
point_poses = zeros(3,number_points) ;

% for all points
for i=1:number_points
    
    %********* Add the 3D position of the point in the vector *********
    for j=1:3
        point_poses(j,i) = data.structure{1,i}.value.X{1,j} ;
    end
    
    %********* Construction of the point_seen_byC structures *********
    % number of camera which see this point 
    seen_by_N = size(data.structure{1,i}.value.observations,2) ;
    
    % for each camera 
    for j=1:seen_by_N
        % get back the right frame number 
        this_cam_key = data.structure{1,i}.value.observations{1,j}.key+1 ;
        this_cam_number = correspondances(1, this_cam_key) ;
        
        % the old information of this cam 
        oldCamInfo  = point_seen_byC{1,this_cam_number} ;
        currentcamInfo = zeros(3,1) ;
        
        % the id of the point (to obtain the 3D position after)
        currentcamInfo(1,1) = i ;
        
        % fill the position of this point in the camera image plane
        for k=1:2
            currentcamInfo(k+1,1) = data.structure{1,i}.value.observations{1,j}.value.x{1,k} ;
        end
        
        % add the new info 
        newCamInfo = [oldCamInfo currentcamInfo] ;
        point_seen_byC{1,this_cam_number} = newCamInfo ;
    end
end

end

% find the number of the frame from the name 
function index = findIndex(camera_name) 
i = 6 ;
thisChar = camera_name(i) ;
name = thisChar ;
while thisChar ~= '.'
    if i==6 
        name = thisChar ;
    else 
        name = [name thisChar] ;
    end
    i=i+1 ;
    thisChar = camera_name(i) ;
end

index =  str2num(name) + 1 ;

end